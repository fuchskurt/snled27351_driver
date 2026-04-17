//! Transport trait and concrete implementations for SNLED27351 bus
//! communication.
//!
//! The driver is generic over any type implementing [`Transport`], allowing
//! the same register logic to run over both SPI and I2C without duplication.
//! Select the appropriate transport for your hardware:
//!
//! - [`SpiTransport`] — wraps a [`SpiBus`](embedded_hal_async::spi::SpiBus), N
//!   CS [`OutputPin`](embedded_hal::digital::OutputPin)s, and one SDB pin.
//! - [`I2cTransport`] — wraps an [`I2c`](embedded_hal_async::i2c::I2c) bus and
//!   N 7-bit device addresses.

use core::fmt::Debug;
#[cfg(feature = "i2c")] pub mod i2c;
#[cfg(feature = "spi")] pub mod spi;
#[cfg(feature = "i2c")] pub use i2c::I2cTransport;
#[cfg(feature = "spi")] pub use spi::SpiTransport;

/// Abstraction over the physical bus used to communicate with one or more
/// SNLED27351 driver chips.
///
/// Implementors handle all framing, CS/address management, and page selection
/// internally so the driver core only deals with register-level semantics.
///
/// # Page selection
///
/// - SPI encodes the page in the command byte header directly.
/// - I2C writes to the command register (`0xFD`) before each register access.
///
/// Both cases are handled transparently inside the transport; the driver
/// core always calls [`write_page`](Transport::write_page) and
/// [`read_reg`](Transport::read_reg) with a plain page number.
pub trait Transport {
    /// The error type returned by bus operations.
    type Error: Debug;

    /// Performs the hardware reset sequence.
    ///
    /// Called once by [`Driver::init`](crate::driver::Driver::init) before
    /// any register writes. For SPI this toggles the SDB pin; for I2C this
    /// is a no-op since reset is handled via the software shutdown register.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn reset(&mut self) -> Result<(), Self::Error>;

    /// Selects `page` on chip `driver_index`, then writes `data` starting at
    /// register `reg`.
    ///
    /// `data` must not exceed
    /// [`PWM_REGISTER_COUNT`](crate::registers::PWM_REGISTER_COUNT) bytes.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Selects `page` on chip `driver_index`, then reads and returns the
    /// single byte at register `reg`.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error>;
}
