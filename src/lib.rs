//! Async embedded-hal driver for the SNLED27351 matrix LED
//! driver IC.
//!
//! Supports both SPI and I2C transports via a common [`transport::Transport`]
//! trait. The driver core in [`driver`] is generic over any
//! [`transport::Transport`] implementation and never duplicates register logic
//! between transports.
//!
//! # Choosing a transport
//!
//! - [`transport::spi::SpiTransport`] — for SPI-connected chips; wraps N
//!   [`embedded_hal_async::spi::SpiDevice`] instances plus one SDB
//!   [`embedded_hal::digital::OutputPin`].
//! - [`transport::i2c::I2cTransport`] — for I2C-connected chips; wraps one
//!   [`embedded_hal_async::i2c::I2c`] bus and N 7-bit addresses.
#![no_std]
#![feature(const_convert, const_index, const_trait_impl)]
pub mod driver;
pub mod led_address;
pub mod registers;
pub mod transport;
