//! SPI transport implementation for the SNLED27351 driver.
//!
//! [`Controller`] owns N [`SpiDevice`] instances (one per chip) each of
//! which manages its own CS line. An additional `S: OutputPin` drives the
//! shared SDB (hardware shutdown) line.
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

use crate::{
    registers::{PATTERN_CMD, PWM_REGISTER_COUNT, READ_CMD, WRITE_CMD},
    transport::Transport,
};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{Operation, SpiDevice};

/// SPI transport for `N` SNLED27351 driver chips on a shared bus.
///
/// `D` is the SPI device type (one per chip, CS-managed by the device itself)
/// and `S` is the SDB pin type.
pub struct Controller<D, S, const N: usize>
where
    D: SpiDevice,
    S: OutputPin,
{
    /// SPI devices, one per driver chip, in driver-index order.
    /// Each device manages its own CS line.
    devices: [D; N],
    /// Shared hardware shutdown / enable pin (high = normal operation).
    sdb:     S,
}

impl<D, S, const N: usize> Controller<D, S, N>
where
    D: SpiDevice,
    S: OutputPin,
{
    /// Creates a new [`Controller`].
    ///
    /// `devices` must be in the same order as the driver indices used by
    /// [`crate::driver::Driver`].
    #[inline]
    pub const fn new(devices: [D; N], sdb: S) -> Self { Self { devices, sdb } }
}

/// Errors that can occur in the SPI transport.
#[derive(Debug)]
#[non_exhaustive]
pub enum TransportError<Error> {
    /// The driver index was out of range.
    InvalidIndex,
    /// The data payload exceeded the maximum transfer size.
    PayloadTooLarge,
    /// An `OutputPin` operation failed.
    Pin,
    /// An SPI bus error occurred.
    Spi(Error),
}

impl<D, S, const N: usize> Transport for Controller<D, S, N>
where
    D: SpiDevice,
    S: OutputPin,
{
    type Error = TransportError<D::Error>;

    #[inline]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error> {
        let device = match self.devices.get_mut(driver_index) {
            Some(dev) => dev,
            None => return Err(TransportError::InvalidIndex),
        };

        let tx = [READ_CMD | PATTERN_CMD | (page & 0x0F), reg, 0x00];
        let mut rx = [0_u8; 3];

        // CS is asserted for the full transaction and released on drop.
        match device.transaction(&mut [Operation::Transfer(&mut rx, &tx)]).await {
            Ok(()) => {},
            Err(error) => return Err(TransportError::Spi(error)),
        }

        rx.get(2).copied().map_or_else(|| Err(TransportError::InvalidIndex), Ok)
    }

    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Pull SDB low to enter hardware shutdown, then release.
        match self.sdb.set_low() {
            Ok(()) => {},
            Err(_) => return Err(TransportError::Pin),
        }
        Timer::after_micros(250).await;
        match self.sdb.set_high() {
            Ok(()) => {},
            Err(_) => return Err(TransportError::Pin),
        }
        // Datasheet requires ~500 µs before the first register access.
        Timer::after_micros(500).await;
        Ok(())
    }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let device = match self.devices.get_mut(driver_index) {
            Some(dev) => dev,
            None => return Err(TransportError::InvalidIndex),
        };

        let mut buf = [0_u8; PWM_REGISTER_COUNT.saturating_add(2)];

        match buf.get_mut(0) {
            Some(cmd) => *cmd = WRITE_CMD | PATTERN_CMD | (page & 0x0F),
            None => return Err(TransportError::InvalidIndex),
        }
        match buf.get_mut(1) {
            Some(reg_slot) => *reg_slot = reg,
            None => return Err(TransportError::InvalidIndex),
        }

        let payload_end = match data.len().checked_add(2) {
            Some(end) => end,
            None => return Err(TransportError::PayloadTooLarge),
        };

        match buf.get_mut(2..payload_end) {
            Some(slot) => slot.copy_from_slice(data),
            None => return Err(TransportError::PayloadTooLarge),
        }

        let out = match buf.get(..payload_end) {
            Some(slice) => slice,
            None => return Err(TransportError::PayloadTooLarge),
        };

        // CS is asserted for the full transaction and released on drop.
        match device.transaction(&mut [Operation::Write(out)]).await {
            Ok(()) => Ok(()),
            Err(error) => Err(TransportError::Spi(error)),
        }
    }
}
