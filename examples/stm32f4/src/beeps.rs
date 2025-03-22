#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

extern crate cs43l22_embedded;
use cs43l22_embedded::{BeepConfig, Cs43l22};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    i2c::I2c,
    i2s::{Config, Format, I2S},
    time::Hertz,
};
use embassy_time::{Duration, Timer};

use fmt::info;

const SAMPLE_RATE: u32 = 48_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = {
        use embassy_stm32::rcc::*;

        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(8),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;

        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL258,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV3),
        });
        config.enable_debug_during_sleep = true;

        config
    };

    let p = embassy_stm32::init(config);

    let mut i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB9, Hertz(100_000), Default::default());
    //configure PD4 as output for reset pin for the CS43L22
    let mut reset = Output::new(p.PD4, Level::Low, Speed::High);
    reset.set_low();
    Timer::after(Duration::from_millis(100)).await;
    reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

    let mut cs43l22 = Cs43l22::new(
        &mut i2c,
        cs43l22_embedded::Config::default(),
        cs43l22_embedded::BeepConfig::default(),
        false,
    )
    .unwrap();

    cs43l22.set_volume_a(100).ok();
    cs43l22.set_volume_b(100).ok();

    let mut dma_buffer = [0u16; 480];

    let mut i2s_config = Config::default();
    i2s_config.format = Format::Data16Channel32;
    i2s_config.standard = embassy_stm32::i2s::Standard::Philips;
    i2s_config.master_clock = true;
    let mut i2s = I2S::new_txonly(
        p.SPI3,
        p.PC12, // sd
        p.PA4,  // ws
        p.PC10, // ck,
        p.PC7,
        p.DMA1_CH7,
        &mut dma_buffer,
        Hertz(SAMPLE_RATE),
        i2s_config,
    );
    i2s.start();

    let pitches = [
        cs43l22_embedded::BeepPitch::C4,
        cs43l22_embedded::BeepPitch::D5,
        cs43l22_embedded::BeepPitch::F5,
        cs43l22_embedded::BeepPitch::F6,
        cs43l22_embedded::BeepPitch::G5,
        cs43l22_embedded::BeepPitch::A5,
        cs43l22_embedded::BeepPitch::B5,
        cs43l22_embedded::BeepPitch::C5,
    ];
    let mut i = 0;

    info!("Playing");
    cs43l22.play().ok();
    loop {
        let beep1 = BeepConfig::default()
            .mix(false)
            .mode(cs43l22_embedded::BeepMode::Continuous)
            .pitch(pitches[i]);
        cs43l22.change_beep_settings(beep1).ok();
        Timer::after(Duration::from_millis(500)).await;
        i = (i + 1) % 8;
    }
}
