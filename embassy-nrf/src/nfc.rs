//! NFC Tag Driver

#![macro_use]

use core::future::poll_fn;
use core::ops::Range;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use nrf_pac::nfct::vals;

use crate::interrupt::InterruptExt;
// use crate::chip::{EASY_DMA_SIZE, FORCE_COPY_BUFFER_SIZE};
use crate::peripherals::NFCT;
use crate::util::slice_in_ram;
use crate::{interrupt, pac, Peripheral};

pub use vals::Bitframesdd as SddPat;
pub use vals::Discardmode as DiscardMode;

/// NFCID1 (aka UID) of different sizes.
#[derive(Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum NfcId {
    /// 4-byte UID.
    SingleSize([u8; 4]),
    /// 7-byte UID.
    DoubleSize([u8; 7]),
    /// 10-byte UID.
    TripleSize([u8; 10]),
}

/// The protocol field to be sent in the `SEL_RES` response byte (b6-b7).
#[derive(Default, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum SelResProtocol {
    /// Configured for Type 2 Tag platform.
    #[default]
    Type2 = 0,
    /// Configured for Type 4A Tag platform, compliant with ISO/IEC_14443.
    Type4A = 1,
    /// Configured for the NFC-DEP Protocol.
    NfcDep = 2,
    /// Configured for the NFC-DEP Protocol and Type 4A Tag platform.
    NfcDepAndType4A = 3,
}

/// Hardware Autocollision config.
#[derive(Clone)]
pub struct AutoCollConfig {
    /// NFCID1 to use during autocollision.
    pub nfcid1: NfcId,
    /// SDD pattern to be sent in `SENS_RES`.
    pub sdd_pat: SddPat,
    /// Platform config to be sent in `SEL_RES`.
    pub plat_conf: u8,
    /// Protocol to be sent in the `SEL_RES` response.
    pub protocol: SelResProtocol,
}

/// Specifies CRC mode.
#[derive(Default, Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub enum CrcMode {
    /// CRC calculation/verification is disabled.
    NoCrc,
    /// Use CRC16.
    #[default]
    Crc16,
}

/// Config for outgoing frames.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct TxdFrameConfig {
    /// Indicates if parity is added to the frame.
    pub parity: bool,
    /// Discarding unused bits at start or end of a frame.
    pub discard_mode: DiscardMode,
    /// Add SoF symbol.
    pub add_sof: bool,
    /// CRC mode for outgoing frames.
    pub crc_mode: CrcMode,
}

impl Default for TxdFrameConfig {
    fn default() -> Self {
        TxdFrameConfig {
            parity: true,
            discard_mode: DiscardMode::DISCARD_START,
            add_sof: true,
            crc_mode: CrcMode::Crc16,
        }
    }
}

/// Config for incoming frames.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct RxdFrameConfig {
    /// Indicates if parity is expected in the received frames.
    pub parity: bool,
    /// Indicates whether SoF symbol is expected.
    pub add_sof: bool,
    /// CRC mode for incoming frames.
    ///
    /// When set to [`CrcMode::NoCrc`] no CRC is expected in a frame, otherwise CRC is verified
    /// and `CRCSTATUS` is updated.
    pub crc_mode: CrcMode,
}

impl Default for RxdFrameConfig {
    fn default() -> Self {
        RxdFrameConfig {
            parity: true,
            add_sof: true,
            crc_mode: CrcMode::Crc16,
        }
    }
}

/// Config for the Frame Delay Timer.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub enum FrameDelayConfig {
    /// Transmission is independent of frame timer and will start when the `STARTTX`` task is
    /// triggered. No timeout.
    FreeRun,
    /// Frame is transmitted a range of 13.56 Mhz clocks.
    ///
    /// The start value should fit in 16 bits, the end value should fit in 20 bits.
    Window(Range<u32>),
    /// Frame is transmitted exactly after a certain amount of 13.56 Mhz clocks.
    ///
    /// The value should fit in 20 bits.
    ExactVal(u32),
    /// Frame is transmitted on a bit grid between a range of 13.56 Mhz clocks.
    ///
    /// The start value should fit in 16 bits, the end value should fit in 20 bits.
    WindowGrid(Range<u32>),
}

impl Default for FrameDelayConfig {
    fn default() -> Self {
        FrameDelayConfig::Window(1152..4096)
    }
}

/// Config for shortcuts the `NFCT` peripheral might take.
#[derive(Default, Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct ShortsConfig {
    /// Activates a shortcut between the `FIELDDETECTED` event and `ACTIVATE` task.
    pub fielddetected_activate: bool,
    /// Activates a shortcut between the `FIELDLOST` event and `SENSE` task.
    pub fieldlost_sense: bool,
    /// Activates a shortcut between the `TXFRAMEEND` event and `ENABLERXDATA` task.
    pub txframeend_enablerxdata: bool,
}

/// Config for the `NFCT` peripheral driver.
#[derive(Clone)]
pub struct Config {
    /// Hardware autocollision resolution config.
    ///
    /// Hardware autocollision resolution is disabled when this is set to [`None`].
    pub autocoll_config: Option<AutoCollConfig>,
    /// Configuration for transmitting frames.
    pub txd_frame_config: TxdFrameConfig,
    /// Configuration for receiving frames.
    pub rxd_frame_config: RxdFrameConfig,
    /// Configuration for the frame delay controller.
    pub frame_delay_config: FrameDelayConfig,
}

/// Interrupt handler.
pub struct InterruptHandler {
    _private: (),
}

impl interrupt::typelevel::Handler<interrupt::typelevel::NFCT> for InterruptHandler {
    unsafe fn on_interrupt() {
        info!("\tNFC Interrupt entry");

        let r = pac::NFCT;
        let mut wake = false;

        if r.events_fielddetected().read() != 0 {
            r.intenclr().write(|w| w.set_fielddetected(true));
            wake = true;
            info!("NFC Interrupt: fielddetected")
        }

        if r.events_fieldlost().read() != 0 {
            r.intenclr().write(|w| w.set_fieldlost(true));
            wake = true;
            info!("NFC Interrupt: fieldlost")
        }

        if r.events_rxframestart().read() != 0 {
            r.intenclr().write(|w| w.set_rxframestart(true));
            r.events_rxframestart().write_value(0);
            wake = true;
            info!("NFC Interrupt: rxframestart")
        }

        if r.events_txframestart().read() != 0 {
            r.intenclr().write(|w| w.set_txframestart(true));
            r.events_txframestart().write_value(0);
            wake = true;
            info!("NFC Interrupt: txframestart")
        }

        if r.events_rxframeend().read() != 0 {
            r.intenclr().write(|w| w.set_rxframeend(true));
            // SAFETY: events_rxframeend cleared by recv_frame
            wake = true;
            info!("NFC Interrupt: rxframeend")
        }

        if r.events_txframeend().read() != 0 {
            r.intenclr().write(|w| w.set_txframeend(true));
            wake = true;
            info!("NFC Interrupt: txframeend")
        }

        if r.events_endrx().read() != 0 {
            r.intenclr().write(|w| w.set_endrx(true));
            wake = true;
            info!("NFC Interrupt: endrx")
        }

        if r.events_endtx().read() != 0 {
            r.intenclr().write(|w| w.set_endtx(true));
            wake = true;
            info!("NFC Interrupt: endtx")
        }

        if r.events_rxerror().read() != 0 {
            r.intenclr().write(|w| w.set_rxerror(true));
            // SAFETY: cleared by recv_frame
            wake = true;
            info!("NFC Interrupt: rxerror")
        }

        if r.events_error().read() != 0 {
            r.intenclr().write(|w| w.set_error(true));
            info!("errorstatus={}", r.errorstatus().read().0);
            wake = true;
            info!("NFC Interrupt: error")
        }

        if r.events_ready().read() != 0 {
            r.intenclr().write(|w| w.set_ready(true));
            // SAFETY: r.events_ready is cleared by user (in `activate()`)
            wake = true;
            info!("NFC Interrupt: ready")
        }

        if r.events_selected().read() != 0 {
            r.intenclr().write(|w| w.set_selected(true));
            wake = true;
            info!("NFC Interrupt: selected")
        }

        if r.events_collision().read() != 0 {
            r.intenclr().write(|w| w.set_collision(true));
            wake = true;
            info!("NFC Interrupt: collision")
        }

        if r.events_started().read() != 0 {
            r.intenclr().write(|w| w.set_started(true));
            r.events_started().write_value(0);
            wake = true;
            info!("NFC Interrupt: started")
        }

        if r.events_collision().read() != 0 {
            r.intenclr().write(|w| w.set_collision(true));
            r.events_collision().write_value(0);
            info!("NFC Interrupt: collision");
        }

        if r.events_autocolresstarted().read() != 0 {
            r.intenclr().write(|w| w.set_autocolresstarted(true));
            // r.events_autocolresstarted().write_value(0);
            info!("NFC Interrupt: autocolresstarted");
        }

        if wake {
            WAKER.wake();
        }

        r.framedelaymax().write(|w| w.0 = FRM_DELAY_MAX.load(Ordering::Relaxed));

        info!("\tNFC Interrupt exit");
    }
}

static WAKER: AtomicWaker = AtomicWaker::new();
static FRM_DELAY_MAX: AtomicU32 = AtomicU32::new(0);

/// NFC error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Rx Error received while waiting for frame
    RxError,
    /// Rx buffer was overrun, increase your buffer size to resolve this
    RxOverrun,
    /// Lost field.
    LostField,
    /// Collision
    Collision,
    /// The buffer is not in data RAM. It's most likely in flash, and nRF's DMA cannot access flash.
    BufferNotInRAM,
}

/// Nfc Tag Read/Writer driver
pub struct NfcT<'d> {
    _p: PeripheralRef<'d, NFCT>,
    rx_buf: [u8; 256],
    tx_buf: [u8; 256],
}

impl<'d> NfcT<'d> {
    /// Create an Nfc Tag driver
    pub fn new(
        _p: impl Peripheral<P = NFCT> + 'd,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::NFCT, InterruptHandler> + 'd,
        config: &Config,
    ) -> Self {
        into_ref!(_p);

        let r = pac::NFCT;

        match &config.frame_delay_config {
            FrameDelayConfig::FreeRun => r.framedelaymode().write(|w| w.0 = 0),
            FrameDelayConfig::Window(range) => {
                FRM_DELAY_MAX.store(range.start, Ordering::Relaxed);
                r.framedelaymin().write(|w| w.0 = range.start);
                r.framedelaymax().write(|w| w.0 = range.end);
                r.framedelaymode().write(|w| w.0 = 1);
            }
            &FrameDelayConfig::ExactVal(val) => {
                FRM_DELAY_MAX.store(val, Ordering::Relaxed);
                r.framedelaymax().write(|w| w.0 = val);
                r.framedelaymode().write(|w| w.0 = 2);
            }
            FrameDelayConfig::WindowGrid(range) => {
                FRM_DELAY_MAX.store(range.start, Ordering::Relaxed);
                r.framedelaymin().write(|w| w.0 = range.start);
                r.framedelaymax().write(|w| w.0 = range.end);
                r.framedelaymode().write(|w| w.0 = 3);
            }
        }

        if let Some(autocoll_config) = config.autocoll_config.as_ref() {
            Self::set_autocoll_cfg(autocoll_config);
        } else {
            r.autocolresconfig().write(|w| w.0 = 0b11u32);
        }

        // errata\
        #[cfg(feature = "nrf52832")]
        unsafe {
            // Errata 57 nrf52832 only
            //(0x40005610 as *mut u32).write_volatile(0x00000005);
            //(0x40005688 as *mut u32).write_volatile(0x00000001);
            //(0x40005618 as *mut u32).write_volatile(0x00000000);
            //(0x40005614 as *mut u32).write_volatile(0x0000003F);

            // Errata 98
            (0x4000568C as *mut u32).write_volatile(0x00038148);
        }

        // TODO: other configs

        r.intenclr().write(|w| w.0 = 0xFFFF_FFFF);

        r.events_autocolresstarted().write_value(0);
        r.events_collision().write_value(0);
        r.events_endrx().write_value(0);
        r.events_endtx().write_value(0);
        r.events_error().write_value(0);
        r.events_fielddetected().write_value(0);
        r.events_fieldlost().write_value(0);
        r.events_ready().write_value(0);
        r.events_rxerror().write_value(0);
        r.events_rxframeend().write_value(0);
        r.events_rxframestart().write_value(0);
        r.events_txframeend().write_value(0);
        r.events_txframestart().write_value(0);
        r.events_selected().write_value(0);
        r.events_started().write_value(0);

        compiler_fence(Ordering::SeqCst);

        interrupt::NFCT.unpend();
        unsafe { interrupt::NFCT.enable() };

        r.intenset().write(|w| {
            w.set_ready(true);
            w.set_txframestart(true);
            w.set_txframeend(true);
            w.set_rxframestart(true);
            w.set_rxframeend(true);
            w.set_fielddetected(true);
            w.set_fieldlost(true);
            w.set_error(true);
            w.set_rxerror(true);
            w.set_selected(true);
            w.set_error(true);
            w.set_collision(true);
            w.set_autocolresstarted(true);
        });
        r.tasks_activate().write_value(1);

        let res = Self {
            _p,
            tx_buf: [0u8; 256],
            rx_buf: [0u8; 256],
        };

        assert!(slice_in_ram(&res.tx_buf), "TX Buf not in ram");
        assert!(slice_in_ram(&res.rx_buf), "RX Buf not in ram");

        res
    }

    /// Checks if field is already present
    pub fn is_field_present(&self) -> bool {
        let r = pac::NFCT;
        return r.fieldpresent().read().fieldpresent();
    }

    /// Blocks until field-detected event is triggered
    pub fn sense(&mut self) {
        let r = pac::NFCT;
        r.tasks_sense().write_value(1);
    }

    /// Blocks until ready event is triggered
    pub async fn wait_for_active(&mut self) {
        let r = pac::NFCT;

        r.intenset().write(|w| w.set_ready(true));
        poll_fn(|cx| {
            let r = pac::NFCT;

            WAKER.register(cx.waker());

            if r.events_fielddetected().read() != 0 {
                r.events_fielddetected().write_value(0);
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    /// Waits for automatic collision detection engine to tell us we've been selected
    pub async fn wait_for_selected(&mut self) -> () {
        trace!("Waiting to be selected");
        poll_fn(|cx| {
            let r = pac::NFCT;

            WAKER.register(cx.waker());

            if r.events_selected().read() != 0 {
                r.events_selected().write_value(0);
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await
    }

    /// Blocks until ready event is triggered
    pub async fn wait_for_coll(&mut self) -> Result<(), Error> {
        let r = pac::NFCT;

        critical_section::with(|_sect| {
            r.events_selected().write_value(0);
            r.events_collision().write_value(0);
            r.events_fieldlost().write_value(0);

            compiler_fence(Ordering::SeqCst);

            r.intenset().write(|w| {
                w.set_selected(true);
                w.set_collision(true);
                w.set_fieldlost(true);
            });

            r.tasks_activate().write_value(1);
        });

        poll_fn(|cx| {
            let r = pac::NFCT;

            WAKER.register(cx.waker());

            critical_section::with(|_sect| {
                if r.events_fieldlost().read() != 0 {
                    r.events_fieldlost().write_value(0);

                    return Poll::Ready(Err(Error::LostField));
                }

                if r.events_collision().read() != 0 {
                    r.events_selected().write_value(0);
                    r.events_collision().write_value(0);

                    return Poll::Ready(Err(Error::Collision));
                }

                if r.events_selected().read() != 0 {
                    r.events_selected().write_value(0);
                    r.events_fieldlost().write_value(0);

                    // clear other events as well
                    r.events_collision().write_value(0);
                    r.events_fielddetected().write_value(0);
                    r.events_rxframestart().write_value(0);
                    r.events_rxframeend().write_value(0);
                    r.events_rxerror().write_value(0);
                    r.events_txframestart().write_value(0);
                    r.events_txframeend().write_value(0);

                    r.framedelaymax().write(|w| w.0 = FRM_DELAY_MAX.load(Ordering::Relaxed));

                    return Poll::Ready(Ok(()));
                }

                Poll::Pending
            })
        })
        .await
    }

    /// Blocks until ready event is triggered
    pub async fn activate(&mut self) {
        let r = pac::NFCT;

        r.intenset().write(|w| w.set_ready(true));
        r.tasks_activate().write_value(1);
        poll_fn(|cx| {
            let r = pac::NFCT;

            WAKER.register(cx.waker());

            if r.events_ready().read() != 0 {
                r.events_ready().write_value(0);
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    pub async fn tx_frame(&mut self, buf: &[u8], bits: u8) -> Result<(), Error> {
        self.tx_frame_with_config(buf, bits, TxdFrameConfig::default()).await
    }

    /// Transmit an NFC frame
    /// `buf` is not pointing to the Data RAM region, an EasyDMA transfer may result in a hard fault or RAM corruption.
    pub async fn tx_frame_with_config(&mut self, buf: &[u8], bits: u8, config: TxdFrameConfig) -> Result<(), Error> {
        //slice_in_ram_or(buf, Error::BufferNotInRAM)?;
        assert!(bits < 8);

        self.tx_buf[..buf.len()].copy_from_slice(buf);

        let r = pac::NFCT;

        let _on_drop = OnDrop::new(|| {
            r.intenclr().write(|w| {
                w.set_txframestart(true);
                w.set_txframeend(true);
                w.set_error(true);
                w.set_fieldlost(true);
            });
        });

        critical_section::with(|_sect| {
            //unsafe { (0x40005010 as *mut u32).write_volatile(1) };

            //Setup DMA
            r.packetptr().write_value(self.tx_buf.as_ptr() as u32);
            r.maxlen().write(|w| w.0 = self.tx_buf.len() as _);
            r.txd().amount().write(|w| {
                w.set_txdatabits(bits);
                w.set_txdatabytes(buf.len() as _);
            });

            r.framedelaymode()
                .write(|w| w.set_framedelaymode(vals::Framedelaymode::WINDOW_GRID));
            r.framedelaymin().write(|w| w.0 = 0x480);
            r.framedelaymax().write(|w| w.0 = 0x1000);

            r.txd().frameconfig().write(|w| {
                w.set_crcmodetx(config.crc_mode == CrcMode::Crc16);
                w.set_discardmode(config.discard_mode);
                w.set_parity(config.parity);
                w.set_sof(config.add_sof);
            });

            r.events_txframestart().write_value(0);
            r.events_txframeend().write_value(0);
            r.events_error().write_value(0);
            r.events_fieldlost().write_value(0);

            r.errorstatus().write(|w| w.set_framedelaytimeout(true));

            compiler_fence(Ordering::SeqCst);

            r.intenset().write(|w| {
                w.set_txframestart(true);
                w.set_txframeend(true);
                w.set_error(true);
                w.set_fieldlost(true);
            });

            compiler_fence(Ordering::SeqCst);

            trace!("nfctagstate before is {}", r.nfctagstate().read().0);

            // Enter TX state
            r.tasks_starttx().write_value(1);
        });

        trace!("nfctagstate after is {}", r.nfctagstate().read().0);

        poll_fn(|cx| {
            let r = pac::NFCT;

            WAKER.register(cx.waker());

            trace!("polling tx {}", r.nfctagstate().read().0);

            critical_section::with(|_sect| {
                r.events_txframestart().write_value(0);

                let mut finished = false;
                if r.events_fieldlost().read() != 0 {
                    trace!("finished tx due to fieldlost");
                    r.events_fieldlost().write_value(0);

                    unsafe { (0x40005010 as *mut u32).write_volatile(1) };

                    return Poll::Ready(Err(Error::LostField));
                }

                if r.events_txframeend().read() != 0 {
                    trace!("clearing txframeend");
                    r.events_txframeend().write_value(0);
                    finished = true;
                }

                if r.events_error().read() != 0 {
                    trace!("clearing error");
                    r.events_error().write_value(0);
                    finished = true;
                }

                if finished {
                    trace!("finished tx");

                    unsafe { (0x40005010 as *mut u32).write_volatile(1) };

                    Poll::Ready(Ok(()))
                } else {
                    trace!("tx pending");

                    Poll::Pending
                }
            })
        })
        .await?;

        Ok(())
    }

    pub async fn tx_frame2(&mut self, buf: &[u8], bits: u8) -> Result<(), Error> {
        let r = pac::NFCT;

        //Setup DMA
        self.tx_buf[..buf.len()].copy_from_slice(buf);
        r.packetptr().write_value(self.tx_buf.as_ptr() as u32);
        r.maxlen().write(|w| w.0 = buf.len() as _);

        // TODO: configure frameconfig
        // Set packet length
        r.txd().amount().write(|w| {
            w.set_txdatabits(bits);
            w.set_txdatabytes(buf.len() as u16 - (bits > 0) as u16);
        });

        let config = TxdFrameConfig {
            parity: false,
            discard_mode: DiscardMode::DISCARD_END,
            add_sof: true,
            crc_mode: CrcMode::Crc16,
        };

        r.txd().frameconfig().write(|w| {
            w.set_crcmodetx(config.crc_mode == CrcMode::Crc16);
            w.set_discardmode(config.discard_mode);
            w.set_parity(config.parity);
            w.set_sof(config.add_sof);
        });

        r.events_txframestart().write_value(0);
        r.events_txframeend().write_value(0);
        r.events_error().write_value(0);
        r.events_fieldlost().write_value(0);

        r.intenset().write(|w| {
            w.set_txframestart(true);
            w.set_txframeend(true);
            w.set_error(true);
            w.set_fieldlost(true);
        });

        // Start starttx task
        compiler_fence(Ordering::SeqCst);
        r.tasks_starttx().write_value(1);

        poll_fn(move |cx| {
            trace!("polling tx");
            let r = pac::NFCT;
            WAKER.register(cx.waker());

            // if r.events_fieldlost().read() != 0 {
            //     return Poll::Ready(Err(Error::LostField));
            // }

            if r.events_txframestart().read() != 0 {
                trace!("Txframstart hit");
                r.events_txframestart().write_value(0);
            }

            if r.events_txframeend().read() != 0 {
                trace!("Txframend hit, should be finished trasmitting");
                r.events_txframeend().write_value(0);
                return Poll::Ready(Ok(()));
            }

            if r.events_error().read() != 0 {
                trace!("Got error?");
                r.events_error().write_value(0);
                // return Poll::Ready(Err(Error::RxError));
            }

            Poll::Pending
        })
        .await
    }

    /// Simplified recv_frame2 which only implements the NFCT state machine as found in the product specs of the NRF5340
    pub async fn recv_frame2<'a>(
        &mut self,
        buf: &'a mut [u8],
        config: RxdFrameConfig,
    ) -> Result<(&'a [u8], u8), Error> {
        let r = pac::NFCT;

        r.rxd().frameconfig().write(|w| {
            w.set_crcmoderx(config.crc_mode == CrcMode::Crc16);
            w.set_parity(config.parity);
            w.set_sof(config.add_sof);
        });

        //Setup DMA
        r.packetptr().write_value(self.rx_buf.as_mut_ptr() as u32);
        r.maxlen().write(|w| w.0 = self.rx_buf.len() as _);

        // Reset and enable the end event
        r.events_rxframeend().write_value(0);
        r.events_rxerror().write_value(0);
        r.events_fieldlost().write_value(0);

        r.intenset().write(|w| {
            w.set_rxframestart(true);
            w.set_rxframeend(true);
            w.set_rxerror(true);
            w.set_fieldlost(true);
        });

        // Start enablerxdata only after configs are finished writing
        compiler_fence(Ordering::SeqCst);
        r.tasks_enablerxdata().write_value(1);

        poll_fn(move |cx| {
            trace!("polling rx");
            let r = pac::NFCT;
            WAKER.register(cx.waker());

            if r.events_fieldlost().read() != 0 {
                return Poll::Ready(Err(Error::LostField));
            }

            if r.events_rxerror().read() != 0 {
                trace!("RXerror got in recv frame, should be back in idle state");
                r.events_rxerror().write_value(0);
                return Poll::Ready(Err(Error::RxError));
            }

            if r.events_rxframeend().read() != 0 {
                trace!("RX Frameend got in recv frame, should have data");
                r.events_rxframeend().write_value(0);
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        })
        .await?;

        let rxd_amount = r.rxd().amount().read();
        let rx_bits = rxd_amount.rxdatabits() as usize;
        let rx_bytes = rxd_amount.rxdatabytes() as usize;

        let buf_bytes = rx_bytes + rx_bits.div_ceil(8);
        buf[..buf_bytes].copy_from_slice(&self.rx_buf[..buf_bytes]);

        Ok((&buf[..buf_bytes], rx_bits as u8))
    }

    pub async fn recv_frame<'a>(&mut self, buf: &'a mut [u8]) -> Result<(&'a [u8], u8), Error> {
        self.recv_frame_with_cfg(buf, Default::default()).await
    }

    /// Waits for a single frame to be loaded into `buf`
    /// `buf` is not pointing to the Data RAM region, an EasyDMA transfer may result in a hard fault or RAM corruption.
    pub async fn recv_frame_with_cfg<'a>(
        &mut self,
        buf: &'a mut [u8],
        config: RxdFrameConfig,
    ) -> Result<(&'a [u8], u8), Error> {
        let r = pac::NFCT;

        let _on_drop = OnDrop::new(|| {
            r.intenclr().write(|w| {
                w.set_rxframestart(true);
                w.set_rxframeend(true);
                w.set_rxerror(true);
                w.set_fieldlost(true);
            });
        });

        // TODO: critical_section is likely not necessary, remove after finishing testing
        critical_section::with(|_sect| {
            r.rxd().frameconfig().write(|w| {
                w.set_crcmoderx(config.crc_mode == CrcMode::Crc16);
                w.set_parity(config.parity);
                w.set_sof(config.add_sof);
            });

            //Setup DMA
            r.packetptr().write_value(self.rx_buf.as_mut_ptr() as u32);
            r.maxlen().write(|w| w.0 = self.rx_buf.len() as _);

            // clear errors
            r.framestatus().rx().write(|w| {
                w.set_crcerror(true);
                w.set_paritystatus(true);
                w.set_overrun(true);
            });
            r.errorstatus().write(|w| {
                w.set_framedelaytimeout(true);
            });

            compiler_fence(Ordering::SeqCst);

            // Reset and enable the end event
            r.events_rxframeend().write_value(0);
            r.events_rxerror().write_value(0);
            r.events_fieldlost().write_value(0);

            compiler_fence(Ordering::SeqCst);

            r.intenset().write(|w| {
                w.set_rxframestart(true);
                w.set_rxframeend(true);
                w.set_rxerror(true);
                w.set_fieldlost(true);
            });

            // Start enablerxdata only after configs are finished writing
            compiler_fence(Ordering::SeqCst);

            // Enter RX state
            r.tasks_enablerxdata().write_value(1);
        });

        trace!("waiting for rx event");

        // Wait for 'rxframeend'/'rxerror' event.
        let (bytes, bits) = poll_fn(move |cx| {
            trace!("polling rx");

            let r = pac::NFCT;

            WAKER.register(cx.waker());

            critical_section::with(|_sect| {
                r.events_txframestart().write_value(0);

                if r.events_fieldlost().read() != 0 {
                    trace!("{} {}", r.events_fielddetected().read(), r.fieldpresent().read().0);
                    if r.events_fielddetected().read() == 0 || !r.fieldpresent().read().fieldpresent() {
                        r.events_fieldlost().write_value(0);
                        r.events_endrx().write_value(0);
                        r.events_rxframeend().write_value(0);
                        r.events_rxerror().write_value(0);
                        return Poll::Ready(Err(Error::LostField));
                    }
                }

                if r.events_rxerror().read() != 0 {
                    r.events_endrx().write_value(0);
                    r.events_rxframeend().write_value(0);
                    r.events_rxerror().write_value(0);

                    let framestatus = r.framestatus().rx().read();
                    let crc_error = framestatus.crcerror();
                    let parity_status = framestatus.paritystatus();
                    let overrun_status = framestatus.overrun();
                    error!(
                        "rx error (crc {} parity {} overrun {})",
                        crc_error, parity_status, overrun_status
                    );
                    r.framestatus().rx().write(|w| {
                        w.set_crcerror(true);
                        w.set_paritystatus(true);
                        w.set_overrun(true);
                    });

                    if overrun_status {
                        return Poll::Ready(Err(Error::RxOverrun));
                    }
                    return Poll::Ready(Err(Error::RxError));
                }

                let mut finished = false;

                if r.events_endrx().read() != 0 {
                    r.events_endrx().write_value(0);
                    finished = true;

                    trace!("cleared endrx");
                }

                if r.events_rxframeend().read() != 0 {
                    r.events_rxframeend().write_value(0);
                    finished = true;

                    trace!("cleared rxframeend");
                }

                if finished {
                    let rxd_amount = r.rxd().amount().read();
                    let amount_read = rxd_amount.rxdatabytes() as usize;
                    let amount_read_bits = rxd_amount.rxdatabits() as usize;
                    let byte_size = (amount_read & 0xFF) + ((amount_read_bits + 7) / 8);

                    trace!("amount {} {} {}", amount_read, amount_read_bits, byte_size);
                    if amount_read > 257 {
                        error!("applying fixup");
                    }

                    Poll::Ready(Ok((byte_size, amount_read_bits as u8)))
                } else {
                    Poll::Pending
                }
            })
        })
        .await?;

        buf[..bytes].copy_from_slice(&self.rx_buf[..bytes]);

        Ok((&buf[..bytes], bits))
    }

    /// Sets the hardware auto collision resolution config.
    fn set_autocoll_cfg(config: &AutoCollConfig) {
        let r = pac::NFCT;

        let nfcid_size = match &config.nfcid1 {
            NfcId::SingleSize(bytes) => {
                r.nfcid1_last().write(|w| w.0 = u32::from_be_bytes(*bytes));

                vals::Nfcidsize::NFCID1SINGLE
            }
            NfcId::DoubleSize(bytes) => {
                let (bytes, chunk) = bytes.split_last_chunk::<4>().unwrap();
                r.nfcid1_last().write(|w| w.0 = u32::from_be_bytes(*chunk));

                let mut chunk = [0u8; 4];
                chunk[1..].copy_from_slice(bytes);
                r.nfcid1_2nd_last().write(|w| w.0 = u32::from_be_bytes(chunk));

                vals::Nfcidsize::NFCID1DOUBLE
            }
            NfcId::TripleSize(bytes) => {
                let (bytes, chunk) = bytes.split_last_chunk::<4>().unwrap();
                r.nfcid1_last().write(|w| w.0 = u32::from_be_bytes(*chunk));

                let (bytes, chunk2) = bytes.split_last_chunk::<3>().unwrap();
                let mut chunk = [0u8; 4];
                chunk[1..].copy_from_slice(chunk2);
                r.nfcid1_2nd_last().write(|w| w.0 = u32::from_be_bytes(chunk));

                let mut chunk = [0u8; 4];
                chunk[1..].copy_from_slice(bytes);
                r.nfcid1_3rd_last().write(|w| w.0 = u32::from_be_bytes(chunk));

                vals::Nfcidsize::NFCID1TRIPLE
            }
        };

        r.sensres().write(|w| {
            w.set_nfcidsize(nfcid_size);
            w.set_bitframesdd(config.sdd_pat);
            w.set_platfconfig(config.plat_conf & 0xF);
        });

        r.selres().write(|w| {
            w.set_protocol(config.protocol as u8);
        });

        r.autocolresconfig().write(|w| w.0 = 0u32);
    }

    /// Sets up shortcuts used by the NFCT peripheral.
    pub fn setup_shorts(&mut self, config: ShortsConfig) {
        let r = pac::NFCT;
        r.shorts().write(|w| {
            w.set_fielddetected_activate(config.fielddetected_activate);
            w.set_fieldlost_sense(config.fieldlost_sense);
            w.set_txframeend_enablerxdata(config.txframeend_enablerxdata);
        });
    }

    /// Requests to enter the `SLEEP_A`` state.
    pub fn sleep(&mut self) {
        let r = pac::NFCT;
        r.tasks_gosleep().write_value(1);
    }

    /// Requests to enter the `IDLE`` state.
    pub fn idle(&mut self) {
        let r = pac::NFCT;
        r.tasks_goidle().write_value(1);
    }
}
