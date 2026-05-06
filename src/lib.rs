//! Async embedded-hal driver for the SNLED27351 matrix LED
//! driver IC.
//!
//! Supports both SPI and I2C transports via a common [`transport::Transport`]
//! trait. The driver core in [`driver`] is generic over any
//! [`transport::Transport`] implementation and never duplicates register logic
//! between transports.
#![no_std]
#![feature(const_convert, const_index, const_trait_impl)]
pub mod driver;
pub mod led_address;
pub mod registers;
pub mod transport;
