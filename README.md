# cs43l22-embedded

A Rust crate for interfacing with the CS43L22 DAC on embedded platforms.

## Overview

The `cs43l22-embedded` crate provides a driver for the CS43L22 DAC. Designed for embedded applications, it allows you to initialize, configure, and control the DAC.

## Features

- Initialization and configuration routines for the CS43L22.
- Control functions for audio.
- Control over data format
- Pretty much everything from the datasheet
- Minimal dependencies.

## Installation

Add the following to your `Cargo.toml` file:

```toml
[dependencies]
cs43l22-embedded = "1.0.0-alpha"
```

## Usage

Here's a basic example to get started:

```rust
// Import the CS43L22 driver
use cs43l22_embedded::Cs43l22;

fn main() {
    // Initialize the driver (adjust parameters as needed)
    let mut cs43l22 = Cs43l22::new(
        &mut i2c_device,
        reset_pin,
        delay_handler,
        cs43l22_embedded::Config::default(),
        cs43l22_embedded::BeepConfig::default(),
        false,
    )
    .unwrap();
    
    // Start audio playback
    cs43l22.play().ok();
    loop{
        // Write pcm audio to the device using I2S
        i2s.write(&audio_buffer);
    }
}
```

Refer to the crate documentation for the available API methods.

## Development

For embedded targets, configure your build environment accordingly. Check the [Embedded Book](https://docs.rust-embedded.org) for additional guidance. Or check the examples for the STM32F407G-DISC1 dev board.

## Notes
Not everything as been tested so alpha stage until I make sure everything works as expected.

## License

Distributed under the GPL-3 License. See `LICENSE` for more information.

