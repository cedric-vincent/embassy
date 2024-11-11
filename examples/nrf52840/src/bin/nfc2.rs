#![no_std]
#![no_main]

use defmt::{assert, assert_eq, panic, todo, *};
use embassy_executor::Spawner;
use embassy_nrf::config::HfclkSource;
use embassy_nrf::nfc::{Config as NfcConfig, NfcId, NfcT};
use embassy_nrf::pac;
use embassy_nrf::{bind_interrupts, nfc};
use {defmt_rtt as _, embassy_nrf as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    NFCT => nfc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    dbg!("Setting up...");
    let config = NfcConfig {
        nfcid1: NfcId::DoubleSize([0x04, 0x68, 0x95, 0x71, 0xFA, 0x5C, 0x64]),
        sdd_pat: nfc::SddPat::SDD00100,
        plat_conf: 0b0000,
        protocol: nfc::SelResProtocol::Type4A,
    };

    let mut nfc = NfcT::new(p.NFCT, Irqs, &config);

    let mut buf = [0u8; 256];

    loop {
        info!("activating");
        nfc.activate().await;

        loop {
            info!("rxing");
            let n = match nfc.receive(&mut buf).await {
                Ok(n) => n,
                Err(e) => {
                    error!("rx error {}", e);
                    break;
                }
            };
            let req = &buf[..n];
            info!("received frame {:02x}", req);

            let mut deselect = false;
            let resp = match req {
                [0xe0, ..] => {
                    info!("Got RATS, tx'ing ATS");
                    &[0x06, 0x77, 0x77, 0x81, 0x02, 0x80][..]
                }
                [0xc2] => {
                    info!("Got deselect!");
                    deselect = true;
                    &[0xc2]
                }
                _ => {
                    info!("Got unknown command!");
                    &[0xFF]
                }
            };

            match nfc.transmit(resp).await {
                Ok(()) => {}
                Err(e) => {
                    error!("tx error {}", e);
                    break;
                }
            }

            if deselect {
                break;
            }
        }
    }
}
