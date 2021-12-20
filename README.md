[![crates.io](https://img.shields.io/crates/v/t67xx.svg)](https://crates.io/crates/t67xx)
[![Docs](https://docs.rs/t67xx/badge.svg)](https://docs.rs/t67xx)

A driver for Telaire T67XX CO2 sensor modules (tested on T6713).

## Example

```rust
use linux_embedded_hal as hal;
use device_driver::ll::LowLevelDevice;
use t67xx::{AbcLogic, T67xxInterface, T67xxLL};

fn main() {
  let i2c = hal::I2cdev::new("/dev/i2c-1").unwrap();
  let delay = hal::Delay {};

  // Interface
  let t67xx_int = T67xxInterface::new(i2c, delay, None);
  // Low-level device
  let mut t67xx = T67xxLL::new(t67xx_int);

  // Enable ABC logic (usually enabled by default)
  t67xx.registers().abc_logic().write(|w| w.abc_logic(AbcLogic::Enabled));

  let status = t67xx.registers().status().read().unwrap();
  println!("{:?}", status);
  let ppm = t67xx.registers().gas_ppm().read().unwrap().gas_ppm();
  println!("CO2: {}ppm", ppm);
}
```

# License

This crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Credits

This crate uses the [device driver toolkit][device-driver].

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, [DerFetzer][team], promises
to intervene to uphold that code of conduct.

[CoC]: https://www.rust-lang.org/policies/code-of-conduct
[team]: https://github.com/DerFetzer
[device-driver]: https://github.com/diondokter/device-driver
