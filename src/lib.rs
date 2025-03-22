#![no_std]
#![deny(rust_2018_idioms)]
#![cfg_attr(docsrs, feature(doc_cfg))]

//! ## Feature flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]

pub(crate) mod fmt;
use embedded_hal::i2c;

const CS43L22_ADDR: u8 = 74;

//Register addresses
const CS43L22_REG_ID: u8 = 0x01;
const CS43L22_REG_POWER_CTL1: u8 = 0x02;
const CS43L22_REG_POWER_CTL2: u8 = 0x04;
const CS43L22_REG_CLOCKING_CTL: u8 = 0x05;
const CS43L22_REG_INTERFACE_CTL1: u8 = 0x06;
/*const CS43L22_REG_INTERFACE_CTL2: u8 = 0x07;
const CS43L22_REG_PASSTHROUGH_A_SELECT: u8 = 0x08;
const CS43L22_REG_PASSTHROUGH_B_SELECT: u8 = 0x09;*/
const CS43L22_REG_ANALOG_ZC_SR_SETT: u8 = 0x0A;
/*const CS43L22_REG_PASSTHROUGH_GANG_CTL: u8 = 0x0C;
const CS43L22_REG_PLAYBACK_CTL1: u8 = 0x0D;*/
const CS43L22_REG_MISC_CTL: u8 = 0x0E;
/*const CS43L22_REG_PLAYBACK_CTL2: u8 = 0x0F;
const CS43L22_REG_PASSTHROUGH_A_VOL: u8 = 0x14;
const CS43L22_REG_PASSTHROUGH_B_VOL: u8 = 0x15;*/
const CS43L22_REG_PCMA_VOL: u8 = 0x1A;
const CS43L22_REG_PCMB_VOL: u8 = 0x1B;
const CS43L22_REG_BEEP_FREQ_ONTIME: u8 = 0x1C;
const CS43L22_REG_BEEP_VOL_OFFTIME: u8 = 0x1D;
const CS43L22_REG_BEEP_TONE_CFG: u8 = 0x1E;
const CS43L22_REG_TONE_CTL: u8 = 0x1F;
const CS43L22_REG_MASTER_A_VOL: u8 = 0x20;
const CS43L22_REG_MASTER_B_VOL: u8 = 0x21;
const CS43L22_REG_HP_A_VOL: u8 = 0x22;
const CS43L22_REG_HP_B_VOL: u8 = 0x23;
/*const CS43L22_REG_SPEAKER_A_VOL: u8 = 0x24;
const CS43L22_REG_SPEAKER_B_VOL: u8 = 0x25;*/
const CS43L22_REG_PCM_CH_SWAP: u8 = 0x26;
const CS43L22_REG_LIMIT_CTL1: u8 = 0x27;
/*const CS43L22_REG_LIMIT_CTL2: u8 = 0x28;
const CS43L22_REG_LIMIT_ATTACK_RATE: u8 = 0x29;
const CS43L22_REG_OVF_CLK_STATUS: u8 = 0x2E;
const CS43L22_REG_BATT_COMP: u8 = 0x2F;
const CS43L22_REG_VP_BATT_LEVEL: u8 = 0x30;
const CS43L22_REG_SPEAKER_STATUS: u8 = 0x31;
const CS43L22_REG_CHARGEPUMP_FREC: u8 = 0x32;*/

#[derive(Debug)]
/// Errors that can occur when using the CS43L22 driver
pub enum Error<I2CError> {
    /// I2C communication error
    I2C(I2CError),
    /// Invalid chip ID
    /// This error is returned when the chip ID read from the CS43L22 is invalid
    InvalidChipID,
}
/// Type alias for the result of a CS43L22 operation
pub type Result<T, I2CError> = core::result::Result<T, Error<I2CError>>;

impl<I2CError> From<I2CError> for Error<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2C(value)
    }
}

/// CS43L22 configuration
/// This struct is used to configure the CS43L22 driver
/// It contains the output device, interface format, word length and volume
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    output_device: OutputDevice,
    interface_format: InterfaceFormat,
    word_length: WordLength,
    treble_cutoff: TrebleCutoff,
    treble_gain: ToneGain,
    bass_cutoff: BassCutoff,
    bass_gain: ToneGain,
    tone_control: bool,
    volume_a: u16,
    volume_b: u16,
}
impl Config {
    /// Create a new CS43L22 configuration
    pub fn new() -> Self {
        Self {
            output_device: OutputDevice::Auto,
            interface_format: InterfaceFormat::I2S,
            word_length: WordLength::W32,
            treble_cutoff: TrebleCutoff::_5kHz,
            treble_gain: ToneGain::_0dB,
            bass_cutoff: BassCutoff::_50Hz,
            bass_gain: ToneGain::_0dB,
            tone_control: false,
            volume_a: 70,
            volume_b: 70,
        }
    }

    /// Set the output device
    pub fn output_device(mut self, output_device: OutputDevice) -> Self {
        self.output_device = output_device;
        self
    }

    /// Set the interface format
    pub fn interface_format(mut self, interface_format: InterfaceFormat) -> Self {
        self.interface_format = interface_format;
        self
    }

    /// Set the word length
    pub fn word_length(mut self, word_length: WordLength) -> Self {
        self.word_length = word_length;
        self
    }

    /// Set the volume
    pub fn volume_a(mut self, volume: u16) -> Self {
        self.volume_a = volume;
        self
    }

    /// Set the volume
    pub fn volume_b(mut self, volume: u16) -> Self {
        self.volume_b = volume;
        self
    }

    /// Set the treble cutoff frequency
    pub fn treble_cutoff(mut self, treble_cutoff: TrebleCutoff) -> Self {
        self.treble_cutoff = treble_cutoff;
        self
    }

    /// Set the treble gain
    pub fn treble_gain(mut self, treble_gain: ToneGain) -> Self {
        self.treble_gain = treble_gain;
        self
    }

    /// Set the bass cutoff frequency
    pub fn bass_cutoff(mut self, bass_cutoff: BassCutoff) -> Self {
        self.bass_cutoff = bass_cutoff;
        self
    }

    /// Set the bass gain
    pub fn bass_gain(mut self, bass_gain: ToneGain) -> Self {
        self.bass_gain = bass_gain;
        self
    }

    /// Set the tone control
    pub fn with_tone_control(mut self, tone_control: bool) -> Self {
        self.tone_control = tone_control;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// CS43L22 driver
/// This struct represents a CS43L22 driver
/// It contains an I2C interface and a configuration
/// It is used to control the CS43L22
/// The driver is generic over the I2C interface
/// The I2C interface must implement the `embedded_hal::i2c::I2c` trait
/// The I2C error type must implement the `core::fmt::Debug` trait

pub struct Cs43l22<I2C> {
    /// I2C interface
    pub i2c: I2C,
    /// CS43L22 configuration
    pub config: Config,
    /// Beep configuration
    pub beep_config: BeepConfig,
    running: bool,
}
/// Type alias for the result of a CS43L22 operation

impl<I2C, I2CError> Cs43l22<I2C>
where
    I2C: i2c::I2c<u8, Error = I2CError>,
    I2CError: i2c::Error,
{
    /// Create a new CS43L22 driver
    /// This function creates a new CS43L22 driver from an I2C interface and initializes the device
    pub fn new(
        i2c: I2C,
        config: Config,
        beep_config: BeepConfig,
        dsp_mode: bool,
    ) -> Result<Self, I2CError> {
        let mut cs43l22 = Self {
            i2c,
            config,
            beep_config,
            running: false,
        };

        //read the chip id
        let mut chip_id = [0u8; 1];
        cs43l22.i2c.write(CS43L22_ADDR, &[CS43L22_REG_ID])?;
        cs43l22.i2c.read(CS43L22_ADDR, &mut chip_id)?;

        if chip_id[0] & 0xF8 != 0xE0 {
            return Err(Error::InvalidChipID);
        }
        trace!("Chip ID: {}", chip_id[0] & 0xF8);

        //Codec OFF
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL1, 0x01])?;
        //Output device
        cs43l22.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_POWER_CTL2, cs43l22.config.output_device.into()],
        )?;
        //Auto clock
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_CLOCKING_CTL, 0x81])?;
        //Generate interface control 1 register value
        // slave mode | inverted sclk | dsp mode | interface format | word length
        let interface_ctl1 = cs43l22.config.word_length.value()
            + (cs43l22.config.interface_format.value() << 2)
            + ((dsp_mode as u8) << 3);

        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_INTERFACE_CTL1, interface_ctl1])?;

        //Disable the analog soft ramp
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_ANALOG_ZC_SR_SETT, 0x00])?;
        //Disable the digital soft ramp
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_MISC_CTL, 0x04])?;
        //Disable the limiter attack level
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_LIMIT_CTL1, 0x00])?;

        //tone configuration
        let tone_cfg = cs43l22.config.tone_control as u8
            | (cs43l22.config.treble_cutoff.value() << 3)
            | (cs43l22.config.bass_cutoff.value() << 1);
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_TONE_CTL, tone_cfg])?;

        //Adjust Bass and Treble levels
        let tone_ctl = cs43l22.config.bass_gain.value() | (cs43l22.config.treble_gain.value() << 4);
        cs43l22
            .i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_TONE_CTL, tone_ctl])?;

        Ok(cs43l22)
    }

    /// Enables the codec and starts the audio playback
    pub fn play(&mut self) -> Result<(), I2CError> {
        if !self.running {
            // Enable the digital soft ramp
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_MISC_CTL, 0x06])?;

            //self.unmute()?;
            // Enable the output device
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL1, 0x9E])?;

            self.running = true;
        }
        Ok(())
    }

    /// Stops the audio playback and disables the codec
    pub fn stop(&mut self) -> Result<(), I2CError> {
        if self.running {
            self.mute()?;
            // Disable the digital soft ramp
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_MISC_CTL, 0x04])?;
            // Power down DAC
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL1, 0x9F])?;
            self.running = false;
        }
        Ok(())
    }

    /// Pauses the audio playback
    pub fn pause(&mut self) -> Result<(), I2CError> {
        if self.running {
            self.mute()?;
            // Put codec in low power mode
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL1, 0x01])?;
            self.running = false;
        }
        Ok(())
    }

    /// Resumes the audio playback
    pub fn resume(&mut self) -> Result<(), I2CError> {
        if !self.running {
            // Enable the digital soft ramp
            self.unmute()?;
            // Exit low power mode
            self.i2c
                .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL1, 0x9E])?;
            self.running = true;
        }
        Ok(())
    }

    /// Mutes the audio playback
    pub fn mute(&mut self) -> Result<(), I2CError> {
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_POWER_CTL2, 0xFF])?;
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_HP_A_VOL, 0x01])?;
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_HP_B_VOL, 0x01])?;

        Ok(())
    }

    /// Unmutes the audio playback
    pub fn unmute(&mut self) -> Result<(), I2CError> {
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_HP_A_VOL, 0x00])?;
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_HP_B_VOL, 0x00])?;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_POWER_CTL2, self.config.output_device.into()],
        )?;
        Ok(())
    }

    /// Changes the output volume for the output A
    pub fn set_volume_a(&mut self, volume: u16) -> Result<(), I2CError> {
        self.config.volume_a = volume;
        self.i2c.write(
            CS43L22_ADDR,
            &[
                CS43L22_REG_MASTER_A_VOL,
                self.convert_volume(self.config.volume_a),
            ],
        )?;
        Ok(())
    }

    /// Changes the output volume for the output B
    pub fn set_volume_b(&mut self, volume: u16) -> Result<(), I2CError> {
        self.config.volume_b = volume;
        self.i2c.write(
            CS43L22_ADDR,
            &[
                CS43L22_REG_MASTER_B_VOL,
                self.convert_volume(self.config.volume_b),
            ],
        )?;
        Ok(())
    }

    /// Fetch the output volume for the output A
    pub fn get_volume_a(&mut self) -> Result<u16, I2CError> {
        Ok(self.config.volume_a)
    }

    /// Fetch the output volume for the output B
    pub fn get_volume_b(&mut self) -> Result<u16, I2CError> {
        Ok(self.config.volume_b)
    }

    /// Changes the volume of the I2S packets for the A
    pub fn set_pcm_volume_a(&mut self, volume: u16) -> Result<(), I2CError> {
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMA_VOL, self.convert_pcm_volume(volume)],
        )?;
        Ok(())
    }

    /// Changes the volume of the I2S packets for the B
    pub fn set_pcm_volume_b(&mut self, volume: u16) -> Result<(), I2CError> {
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMB_VOL, self.convert_pcm_volume(volume)],
        )?;
        Ok(())
    }

    /// Mutes the I2S stream
    pub fn pcm_mute(&mut self) -> Result<(), I2CError> {
        let mut current_volume_a = [0u8; 1];
        let mut current_volume_b = [0u8; 1];
        self.i2c
            .write_read(CS43L22_ADDR, &[CS43L22_REG_PCMA_VOL], &mut current_volume_a)?;
        self.i2c
            .write_read(CS43L22_ADDR, &[CS43L22_REG_PCMB_VOL], &mut current_volume_b)?;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMA_VOL, current_volume_a[0] | 0x80],
        )?;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMB_VOL, current_volume_b[0] | 0x80],
        )?;
        Ok(())
    }

    /// Unmutes the I2S stream
    pub fn pcm_unmute(&mut self) -> Result<(), I2CError> {
        let mut current_volume_a = [0u8; 1];
        let mut current_volume_b = [0u8; 1];
        self.i2c
            .write_read(CS43L22_ADDR, &[CS43L22_REG_PCMA_VOL], &mut current_volume_a)?;
        self.i2c
            .write_read(CS43L22_ADDR, &[CS43L22_REG_PCMB_VOL], &mut current_volume_b)?;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMA_VOL, current_volume_a[0] & 0x7F],
        )?;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCMB_VOL, current_volume_b[0] & 0x7F],
        )?;
        Ok(())
    }

    /// Sets the beep tone configuration
    pub fn setup_beep(&mut self) -> Result<(), I2CError> {
        //read current value of the beep tone configuration
        let mut tone_cfg = [0u8; 1];
        self.i2c
            .write_read(CS43L22_ADDR, &[CS43L22_REG_BEEP_TONE_CFG], &mut tone_cfg)?;

        tone_cfg[0] = tone_cfg[0] & 0x0F;

        let frec_ontime_value =
            self.beep_config.on_time.value() | (self.beep_config.pitch.value() << 4) + tone_cfg[0];
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_BEEP_FREQ_ONTIME, frec_ontime_value],
        )?;
        let vol_offtime_value = self.convert_beep_volume(self.beep_config.volume)
            | (self.beep_config.off_time.value() << 5);
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_BEEP_VOL_OFFTIME, vol_offtime_value],
        )?;

        //TODO: Implement tone configuration
        let tone_cfg_value =
            (self.beep_config.mode.value() << 6) | ((!self.beep_config.mix as u8) << 5);
        self.i2c
            .write(CS43L22_ADDR, &[CS43L22_REG_BEEP_TONE_CFG, tone_cfg_value])?;

        Ok(())
    }

    /// Changes the beep tone configuration and starts the beep
    pub fn change_beep_settings(&mut self, beep_config: BeepConfig) -> Result<(), I2CError> {
        self.beep_config = beep_config;
        self.setup_beep()
    }

    /// Allow the I2S stream to mix with other sounds
    pub fn set_pcm_mix(&mut self, mix: PCMmix) -> Result<(), I2CError> {
        let pcm_channel_swap_val = mix.value() << 4;
        self.i2c.write(
            CS43L22_ADDR,
            &[CS43L22_REG_PCM_CH_SWAP, pcm_channel_swap_val],
        )?;
        Ok(())
    }

    //utils
    fn convert_volume(&self, volume: u16) -> u8 {
        let mut volume = (volume * 255 / 100) as u8;
        if volume > 0xE6 {
            volume = volume - 0xE7;
        } else {
            volume = volume + 0x19;
        }
        volume
    }
    fn convert_pcm_volume(&self, volume: u16) -> u8 {
        let mut volume = (volume * 114 / 100) as u8;
        if volume > 0x67 {
            volume = volume - 0x68;
        } else {
            volume = volume + 0x19;
        }
        volume
    }
    fn convert_beep_volume(&self, volume: u16) -> u8 {
        let mut volume = (volume * 62 / 100) as u8;
        if volume > 0x36 {
            volume = volume - 0x37;
        } else {
            volume = volume + 0x07;
        }
        volume
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The output device of the CS43L22
/// This is used to select the output device of the CS43L22
/// on the register 0x04
pub enum OutputDevice {
    /// Speaker
    Speaker,
    /// Headphone
    Headphone,
    /// Both
    Both,
    /// Auto
    Auto,
}

impl OutputDevice {
    fn value(self) -> u8 {
        match self {
            Self::Speaker => 0xFA,
            Self::Headphone => 0xAF,
            Self::Both => 0xAA,
            Self::Auto => 0x05,
        }
    }
}

impl From<OutputDevice> for u8 {
    fn from(device: OutputDevice) -> Self {
        device.value()
    }
}

/// Interface format of the I2S stream
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterfaceFormat {
    /// DSP Mode
    LeftJustified,
    /// I2S
    I2S,
    /// Right Justified
    RightJustified,
}
impl InterfaceFormat {
    fn value(self) -> u8 {
        match self {
            Self::LeftJustified => 0x00,
            Self::I2S => 0x01,
            Self::RightJustified => 0x02,
        }
    }
}
impl From<InterfaceFormat> for u8 {
    fn from(interface_format: InterfaceFormat) -> Self {
        interface_format.value()
    }
}

/// Word length of the I2S stream
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WordLength {
    /// 16 bits
    W16,
    /// 20 bits
    W20,
    /// 24 bits
    W24,
    /// 32 bits
    W32,
}
impl WordLength {
    fn value(self) -> u8 {
        match self {
            Self::W16 => 0x03,
            Self::W20 => 0x02,
            Self::W24 => 0x01,
            Self::W32 => 0x00,
        }
    }
}
impl From<WordLength> for u8 {
    fn from(word_length: WordLength) -> Self {
        word_length.value()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The output device of the CS43L22
/// This is used to select the output device of the CS43L22
/// on the register 0x04
pub enum BeepPitch {
    C4,
    C5,
    D5,
    E5,
    F5,
    G5,
    A5,
    B5,
    C6,
    D6,
    E6,
    F6,
    G6,
    A6,
    B6,
    C7,
}

impl BeepPitch {
    fn value(self) -> u8 {
        match self {
            Self::C4 => 0b0000,
            Self::C5 => 0b0001,
            Self::D5 => 0b0010,
            Self::E5 => 0b0011,
            Self::F5 => 0b0100,
            Self::G5 => 0b0101,
            Self::A5 => 0b0110,
            Self::B5 => 0b0111,
            Self::C6 => 0b1000,
            Self::D6 => 0b1001,
            Self::E6 => 0b1010,
            Self::F6 => 0b1011,
            Self::G6 => 0b1100,
            Self::A6 => 0b1101,
            Self::B6 => 0b1110,
            Self::C7 => 0b1111,
        }
    }
}

impl From<BeepPitch> for u8 {
    fn from(pitch: BeepPitch) -> Self {
        pitch.value()
    }
}

/// Time the beep is on
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BeepOnTime {
    /// 86ms
    Ms86,
    /// 430ms
    Ms430,
    /// 780ms
    Ms780,
    /// 1200ms
    Ms1200,
    /// 1500ms
    Ms1500,
    /// 1800ms
    Ms1800,
    /// 2200ms
    Ms2200,
    /// 2500ms
    Ms2500,
    /// 2800ms
    Ms2800,
    /// 3200ms
    Ms3200,
    /// 3500ms
    Ms3500,
    /// 3800ms
    Ms3800,
    /// 4200ms
    Ms4200,
    /// 4500ms
    Ms4500,
    /// 4800ms
    Ms4800,
    /// 5200ms
    Ms5200,
}

impl BeepOnTime {
    fn value(self) -> u8 {
        match self {
            Self::Ms86 => 0b0000,
            Self::Ms430 => 0b0001,
            Self::Ms780 => 0b0010,
            Self::Ms1200 => 0b0011,
            Self::Ms1500 => 0b0100,
            Self::Ms1800 => 0b0101,
            Self::Ms2200 => 0b0110,
            Self::Ms2500 => 0b0111,
            Self::Ms2800 => 0b1000,
            Self::Ms3200 => 0b1001,
            Self::Ms3500 => 0b1010,
            Self::Ms3800 => 0b1011,
            Self::Ms4200 => 0b1100,
            Self::Ms4500 => 0b1101,
            Self::Ms4800 => 0b1110,
            Self::Ms5200 => 0b1111,
        }
    }
}

impl From<BeepOnTime> for u8 {
    fn from(time: BeepOnTime) -> Self {
        time.value()
    }
}

/// Time the beep is off
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BeepOffTime {
    /// 1230ms
    Ms1230,
    /// 2580ms
    Ms2580,
    /// 3900ms
    Ms3900,
    /// 5220ms
    Ms5220,
    /// 6600ms
    Ms6600,
    /// 8050ms
    Ms8050,
    /// 9350ms
    Ms9350,
    /// 10800ms
    Ms10800,
}

impl BeepOffTime {
    fn value(self) -> u8 {
        match self {
            Self::Ms1230 => 0b000,
            Self::Ms2580 => 0b001,
            Self::Ms3900 => 0b010,
            Self::Ms5220 => 0b011,
            Self::Ms6600 => 0b100,
            Self::Ms8050 => 0b101,
            Self::Ms9350 => 0b110,
            Self::Ms10800 => 0b111,
        }
    }
}

impl From<BeepOffTime> for u8 {
    fn from(time: BeepOffTime) -> Self {
        time.value()
    }
}

/// Beep mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BeepMode {
    /// Beep is off
    Off,
    /// Beep is on for a single frame
    Single,
    /// Beep is on for multiple frames
    Multiple,
    /// Beep is on continuously
    Continuous,
}
impl BeepMode {
    fn value(self) -> u8 {
        match self {
            Self::Off => 0b00,
            Self::Single => 0b01,
            Self::Multiple => 0b10,
            Self::Continuous => 0b11,
        }
    }
}

impl From<BeepMode> for u8 {
    fn from(mode: BeepMode) -> Self {
        mode.value()
    }
}

/// Beep configuration
/// This struct is used to configure the beep tone of the CS43L22
/// It contains the pitch, on time, off time, mode, mix and volume
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BeepConfig {
    pitch: BeepPitch,
    on_time: BeepOnTime,
    off_time: BeepOffTime,
    mode: BeepMode,
    mix: bool,
    volume: u16,
}

impl BeepConfig {
    /// Create a new CS43L22 configuration
    pub fn new() -> Self {
        Self {
            pitch: BeepPitch::C4,
            on_time: BeepOnTime::Ms86,
            off_time: BeepOffTime::Ms1230,
            mode: BeepMode::Off,
            mix: false,
            volume: 70,
        }
    }

    /// Set the pitch
    pub fn pitch(mut self, pitch: BeepPitch) -> Self {
        self.pitch = pitch;
        self
    }

    /// Set the on time
    pub fn on_time(mut self, on_time: BeepOnTime) -> Self {
        self.on_time = on_time;
        self
    }

    /// Set the off time
    pub fn off_time(mut self, off_time: BeepOffTime) -> Self {
        self.off_time = off_time;
        self
    }

    /// Set the volume
    pub fn volume(mut self, volume: u16) -> Self {
        self.volume = volume;
        self
    }

    /// Set the mode
    pub fn mode(mut self, mode: BeepMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set the mix
    pub fn mix(mut self, mix: bool) -> Self {
        self.mix = mix;
        self
    }
}

impl Default for BeepConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Treble cutoff frequency
/// This is used to set the treble cutoff frequency of the CS43L22
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TrebleCutoff {
    /// 5kHz
    _5kHz,
    /// 7kHz
    _7kHz,
    /// 10kHz
    _10kHz,
    /// 15kHz
    _15kHz,
}

impl TrebleCutoff {
    fn value(self) -> u8 {
        match self {
            Self::_5kHz => 0b00,
            Self::_7kHz => 0b01,
            Self::_10kHz => 0b10,
            Self::_15kHz => 0b11,
        }
    }
}

impl From<TrebleCutoff> for u8 {
    fn from(cutoff: TrebleCutoff) -> Self {
        cutoff.value()
    }
}

/// Bass cutoff frequency
/// This is used to set the bass cutoff frequency of the CS43L22
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BassCutoff {
    /// 50Hz
    _50Hz,
    /// 100Hz
    _100Hz,
    /// 200Hz
    _200Hz,
    /// 250Hz
    _250Hz,
}

impl BassCutoff {
    fn value(self) -> u8 {
        match self {
            Self::_50Hz => 0b00,
            Self::_100Hz => 0b01,
            Self::_200Hz => 0b10,
            Self::_250Hz => 0b11,
        }
    }
}

impl From<BassCutoff> for u8 {
    fn from(cutoff: BassCutoff) -> Self {
        cutoff.value()
    }
}

/// Tone gain
/// This is used to set the treble and bass gain of the CS43L22
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ToneGain {
    _12dB,
    _10_5dB,
    _9dB,
    _7_5dB,
    _6dB,
    _4_5dB,
    _3dB,
    _1_5dB,
    _0dB,
    Min1_5dB,
    Min3dB,
    Min4_5dB,
    Min6dB,
    Min7_5dB,
    Min9dB,
    Min10_5dB,
}

impl ToneGain {
    fn value(self) -> u8 {
        match self {
            Self::_12dB => 0b0000,
            Self::_10_5dB => 0b0001,
            Self::_9dB => 0b0010,
            Self::_7_5dB => 0b0011,
            Self::_6dB => 0b0100,
            Self::_4_5dB => 0b0101,
            Self::_3dB => 0b0110,
            Self::_1_5dB => 0b0111,
            Self::_0dB => 0b1000,
            Self::Min1_5dB => 0b1001,
            Self::Min3dB => 0b1010,
            Self::Min4_5dB => 0b1011,
            Self::Min6dB => 0b1100,
            Self::Min7_5dB => 0b1101,
            Self::Min9dB => 0b1110,
            Self::Min10_5dB => 0b1111,
        }
    }
}

impl From<ToneGain> for u8 {
    fn from(gain: ToneGain) -> Self {
        gain.value()
    }
}

/// PCM mix
/// This is used to set the PCM mix of the CS43L22
/// In summary, it is used to set the gain of the left and right channels
/// of the I2S stream
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PCMmix {
    /// Full left channel
    Left,
    /// (Left + Right) / 2
    Half,
    /// Full right channel
    Right,
}

impl PCMmix {
    fn value(self) -> u8 {
        match self {
            Self::Left => 0b00,
            Self::Half => 0b01,
            Self::Right => 0b11,
        }
    }
}

impl From<PCMmix> for u8 {
    fn from(mix: PCMmix) -> Self {
        mix.value()
    }
}
