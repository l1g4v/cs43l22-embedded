#![no_std]
#![no_main]

mod fmt;
use core::f32::consts::PI;

use fmt::info;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    i2c::I2c,
    i2s::{Config, Format, I2S},
    time::Hertz,
};
use embassy_time::{Duration, Timer};

use cs43l22_embedded::Cs43l22;
use micromath::F32Ext;

const SAMPLE_RATE: u32 = 48_000;
const FRECUENCY: u32 = 375;

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

        // I2S PLL
        // 8Mhz / 8 = 1Mhz
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
        cs43l22_embedded::Config::default()
            .output_device(cs43l22_embedded::OutputDevice::Auto)
            .with_tone_control(true)
            .treble_cutoff(cs43l22_embedded::TrebleCutoff::_5kHz)
            .treble_gain(cs43l22_embedded::ToneGain::_12dB),
        cs43l22_embedded::BeepConfig::default()
            .mix(true)
            .mode(cs43l22_embedded::BeepMode::Off)
            .pitch(cs43l22_embedded::BeepPitch::F6),
        false,
    )
    .unwrap();

    cs43l22.set_volume_a(100).ok();
    cs43l22.set_volume_b(100).ok();

    // stereo wavetable generation
    let mut wavetable = [0u16; (2 * SAMPLE_RATE / FRECUENCY) as usize];
    let sample_dt = FRECUENCY as f32 / SAMPLE_RATE as f32;
    for i in 0..(SAMPLE_RATE / FRECUENCY) as usize {
        let sinval = (2.0 * PI * i as f32 * sample_dt).sin(); // + (2.0* PI * i as f32*harmonic_dt).sin();

        let val = (sinval * 16383.75) as u16;
        //store half of the binary value in one channel and the other half in the other channel
        wavetable[i * 2] = val;
        wavetable[i * 2 + 1] = val;
    }

    let mut dma_buffer = [0u16; (2 * SAMPLE_RATE / FRECUENCY) as usize];

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

    info!("Playing");
    cs43l22.play().ok();
    loop {
        i2s.write(&wavetable).await.ok();
    }
}
