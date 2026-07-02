//! Async embedded-hal driver for the SNLED27351 matrix LED
//! driver IC.
//!
//! Supports both SPI and I2C transports via a common [`transport::Transport`]
//! trait. The driver core in [`driver`] is generic over any
//! [`transport::Transport`] implementation and never duplicates register logic
//! between transports.
#![no_std]
pub mod driver;
pub mod led_address;
pub mod registers;
pub mod transport;
pub use driver::{Driver, Led, LedStatusMap};
pub use led_address::pwm_register;
pub use transport::{Error as TransportError, NoSdb, Transport};
