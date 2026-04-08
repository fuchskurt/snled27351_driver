//! I2C transport implementation for the SNLED27351 driver.
//!
//! Uses [`embedded_hal_async::i2c::I2c`] with one 7-bit device address per
//! driver chip. Page selection is performed by writing to the command register
//! (0xFD) before each access, as required by the SNLED27351 I2C protocol.

use crate::{registers::CONFIGURE_CMD_PAGE, transport::Transport};
use embedded_hal_async::i2c::I2c;

/// I2C transport for `N` SNLED27351 driver chips on a shared bus.
///
/// All chips share one [`I2c`] bus instance and are distinguished by their
/// 7-bit addresses in `addrs`.
pub struct I2cTransport<B, const N: usize>
where
    B: I2c,
{
    /// The I2C bus peripheral.
    bus: B,
    /// 7-bit I2C addresses, one per driver chip, in driver-index order.
    addrs: [u8; N],
}

impl<B, const N: usize> I2cTransport<B, N>
where
    B: I2c,
{
    /// Creates a new [`I2cTransport`] from the given I2C bus and address list.
    ///
    /// `addrs` must be in the same order as the driver indices used by
    /// [`Driver`](crate::driver::Driver).
    pub const fn new(bus: B, addrs: [u8; N]) -> Self { Self { bus, addrs } }
}

/// Errors that can occur in the I2C transport.
#[derive(Debug)]
pub enum I2cTransportError<E> {
    /// An I2C bus error occurred.
    I2c(E),
    /// The driver index was out of range.
    InvalidIndex,
    /// The data payload exceeded the maximum transfer size.
    PayloadTooLarge,
}

impl<B, const N: usize> Transport for I2cTransport<B, N>
where
    B: I2c,
{
    type Error = I2cTransportError<B::Error>;

    /// I2C chips are reset via the software shutdown register in
    /// [`Driver::init`]; no hardware pin toggle is needed here.
    async fn reset(&mut self) -> Result<(), Self::Error> { Ok(()) }

    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        use crate::registers::PWM_REGISTER_COUNT;

        if data.len() > PWM_REGISTER_COUNT {
            return Err(I2cTransportError::PayloadTooLarge);
        }

        let Some(&addr) = self.addrs.get(driver_index) else {
            return Err(I2cTransportError::InvalidIndex);
        };

        // Select the page via the command register.
        self.bus.write(addr, &[CONFIGURE_CMD_PAGE, page]).await.map_err(I2cTransportError::I2c)?;

        // Write register address followed by data in one transaction.
        let mut buf = [0_u8; PWM_REGISTER_COUNT.saturating_add(1)];

        let Some(reg_slot) = buf.get_mut(0) else {
            return Err(I2cTransportError::InvalidIndex);
        };
        *reg_slot = reg;

        let payload_end = data.len().saturating_add(1);
        let Some(payload) = buf.get_mut(1..payload_end) else {
            return Err(I2cTransportError::PayloadTooLarge);
        };
        payload.copy_from_slice(data);

        let Some(out) = buf.get(..payload_end) else {
            return Err(I2cTransportError::PayloadTooLarge);
        };

        self.bus.write(addr, out).await.map_err(I2cTransportError::I2c)
    }

    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error> {
        let Some(&addr) = self.addrs.get(driver_index) else {
            return Err(I2cTransportError::InvalidIndex);
        };

        // Select the page via the command register.
        self.bus.write(addr, &[CONFIGURE_CMD_PAGE, page]).await.map_err(I2cTransportError::I2c)?;

        // Write the register address, then read one byte back.
        let mut buf = [0_u8; 1];
        self.bus.write_read(addr, &[reg], &mut buf).await.map_err(I2cTransportError::I2c)?;
        Ok(buf[0])
    }
}
