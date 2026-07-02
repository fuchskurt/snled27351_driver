//! SPI transport implementation for the SNLED27351 driver.
//!
//! [`Controller`] owns N [`SpiDevice`] instances (one per chip) each of
//! which manages its own CS line. An additional `S: OutputPin` drives the
//! shared SDB (hardware shutdown) line; use
//! [`NoSdb`](crate::transport::NoSdb) if SDB is strapped high in hardware.
//!
//! The maximum supported SCK frequency is 4 MHz (datasheet section 5).
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
    /// [`crate::driver::Driver`]. `sdb` is the shared SDB line; pass
    /// [`NoSdb`](crate::transport::NoSdb) if it is strapped high in
    /// hardware.
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
        let Some(device) = self.devices.get_mut(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };

        let tx = [READ_CMD | PATTERN_CMD | (page & 0x0F), reg, 0x00];
        let mut rx = [0_u8; 3];

        // CS is asserted for the full transaction and released on drop.
        device.transaction(&mut [Operation::Transfer(&mut rx, &tx)]).await.map_err(TransportError::Spi)?;

        // The chip drives the response on the third byte of the transfer.
        let [_, _, value] = rx;
        Ok(value)
    }

    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Cycle SDB through hardware sleep and back to reach a known power
        // state (registers are retained; `init` reprograms them anyway).
        // Datasheet: entering hardware sleep takes 16 µs.
        self.set_sdb(false)?;
        Timer::after_micros(250).await;
        self.set_sdb(true)?;
        // Datasheet: 128 µs wakeup time before the first register access.
        Timer::after_micros(500).await;
        Ok(())
    }

    #[inline]
    fn set_sdb(&mut self, enable: bool) -> Result<(), Self::Error> {
        let result = if enable { self.sdb.set_high() } else { self.sdb.set_low() };
        if result.is_err() {
            return Err(TransportError::Pin);
        }
        Ok(())
    }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let Some(device) = self.devices.get_mut(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };

        let header = [WRITE_CMD | PATTERN_CMD | (page & 0x0F), reg];

        // CS stays asserted across both operations and is released on drop,
        // so the chip sees a single continuous frame: header, then payload.
        device.transaction(&mut [Operation::Write(&header), Operation::Write(data)]).await.map_err(TransportError::Spi)
    }
}
