//! Async embedded-hal driver for the SNLED27351 / CKLED2001 matrix LED
//! driver IC.
//!
//! Supports both SPI and I2C transports via a common [`Transport`] trait.
//! The driver core in [`driver`] is generic over any [`Transport`]
//! implementation so register logic is never duplicated between transports.
//!
//! # Choosing a transport
//!
//! - [`transport::spi::SpiTransport`] — for SPI-connected chips; wraps N
//!   [`embedded_hal_async::spi::SpiDevice`] instances plus one SDB
//!   [`embedded_hal::digital::OutputPin`].
//! - [`transport::i2c::I2cTransport`] — for I2C-connected chips; wraps one
//!   [`embedded_hal_async::i2c::I2c`] bus and N 7-bit addresses.
#![no_std]
#![deny(warnings)]
#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![feature(const_convert)]
#![feature(const_index)]
#![feature(const_trait_impl)]
#![expect(clippy::implicit_return, clippy::single_call_fn, reason = "Implementation specific ignored lints")]
pub mod driver;
pub mod led_address;
pub mod registers;
pub mod transport;
