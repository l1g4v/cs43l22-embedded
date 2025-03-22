# cs43l22-embedded

A Rust crate for interfacing with the CS43L22 audio codec on embedded platforms.

## Overview

The `cs43l22-embedded` crate provides a driver for the CS43L22 audio codec. Designed for embedded applications, it allows you to initialize, configure, and control audio playback.

## Features

- Initialization and configuration routines for the CS43L22.
- Control functions for audio playback.
- Minimal dependencies, ideal for no_std environments.

## Installation

Add the following to your `Cargo.toml` file:

```toml
[dependencies]
cs43l22-embedded = "0.1.0"
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

For embedded targets, configure your build environment accordingly. Check the [Rust Embedded Book](https://docs.rust-embedded.org) for additional guidance.

## License

Distributed under the GPL-3 License. See `LICENSE` for more information.

