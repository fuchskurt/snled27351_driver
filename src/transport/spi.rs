//! SPI transport implementation for the SNLED27351 driver.
//!
//! [`SpiTransport`] owns one [`SpiBus`] and N chip-select
//! [`OutputPin`]s, asserting the correct CS around every transfer.
//! An additional `S: OutputPin` drives the shared SDB (hardware shutdown)
//! line.
//!
//! # SPI frame format
//!
//! Each write transfer is:
//! - Byte 0: `WRITE_CMD | PATTERN_CMD | (page & 0x0F)`
//! - Byte 1: register address
//! - Bytes 2+: data payload (chip auto-increments address)
//!
//! Each read transfer is:
//! - Byte 0: `READ_CMD | PATTERN_CMD | (page & 0x0F)`
//! - Byte 1: register address
//! - Byte 2: dummy — chip drives the response byte here

use embedded_hal::digital::OutputPin;
use crate::{
    registers::{PATTERN_CMD, PWM_REGISTER_COUNT, READ_CMD, WRITE_CMD},
    transport::Transport,
};
use embedded_hal_async::spi::SpiBus;

/// SPI transport for `N` SNLED27351 driver chips on a shared bus.
///
/// `B` is the SPI bus, `C` is the CS pin type (one per chip), and `S` is
/// the SDB pin type. All three are held by value so the transport owns the
/// full hardware resources it needs.
pub struct SpiTransport<B, C, S, const N: usize>
where
    B: SpiBus,
    C: OutputPin,
    S: OutputPin,
{
    /// SPI bus shared across all chips.
    bus: B,
    /// Chip-select outputs, one per driver chip, in driver-index order.
    cs: [C; N],
    /// Shared hardware shutdown / enable pin (high = normal operation).
    sdb: S,
}

impl<B, C, S, const N: usize> SpiTransport<B, C, S, N>
where
    B: SpiBus,
    C: OutputPin,
    S: OutputPin,
{
    /// Creates a new [`SpiTransport`].
    ///
    /// `cs` must be in the same order as the driver indices used by
    /// [`crate::driver::Driver`]. All CS pins are initialized high
    /// (deasserted) on construction.
    #[inline]
    pub const fn new(bus: B, cs: [C; N], sdb: S) -> Self { Self { bus, cs, sdb } }
}

/// Errors that can occur in the SPI transport.
#[derive(Debug)]
#[non_exhaustive]
pub enum TransportError<E> {
    /// An SPI bus error occurred.
    Spi(E),
    /// The driver index was out of range.
    InvalidIndex,
    /// The data payload exceeded the maximum transfer size.
    PayloadTooLarge,
    /// An `OutputPin` operation failed.
    Pin,
}

impl<B, C, S, const N: usize> Transport for SpiTransport<B, C, S, N>
where
    B: SpiBus,
    C: OutputPin,
    S: OutputPin,
{
    type Error = TransportError<B::Error>;

    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Pull SDB low to enter hardware shutdown, then release.
        self.sdb.set_low().map_err(|_ignored| TransportError::Pin)?;
        embassy_time::Timer::after_micros(250).await;
        self.sdb.set_high().map_err(|_ignored| TransportError::Pin)?;
        // Datasheet requires ~500 µs before the first register access.
        embassy_time::Timer::after_micros(500).await;
        Ok(())
    }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let mut buf = [0_u8; PWM_REGISTER_COUNT.saturating_add(2)];
        let Some(cmd) = buf.get_mut(0) else { return Err(TransportError::InvalidIndex) };
        *cmd = WRITE_CMD | PATTERN_CMD | (page & 0x0F);
        let Some(reg_slot) = buf.get_mut(1) else { return Err(TransportError::InvalidIndex) };
        *reg_slot = reg;
        let payload_end = data.len().saturating_add(2);
        let Some(payload) = buf.get_mut(2..payload_end) else {
            return Err(TransportError::PayloadTooLarge);
        };
        payload.copy_from_slice(data);

        let Some(cs) = self.cs.get_mut(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };
        cs.set_low().map_err(|_ignored| TransportError::Pin)?;

        let result = self.bus.write(buf.get(..payload_end).unwrap_or(&[])).await.map_err(TransportError::Spi);

        let _ = self.cs.get_mut(driver_index).map(OutputPin::set_high);
        result
    }

    #[inline]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error> {
        let tx = [READ_CMD | PATTERN_CMD | (page & 0x0F), reg, 0x00];
        let mut rx = [0_u8; 3];

        let Some(cs) = self.cs.get_mut(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };
        cs.set_low().map_err(|_ignored| TransportError::Pin)?;

        let result = self.bus.transfer(&mut rx, &tx).await.map_err(TransportError::Spi);

        let _ = self.cs.get_mut(driver_index).map(OutputPin::set_high);
        result?;

        rx.get(2).copied().ok_or(TransportError::InvalidIndex)
    }
}
