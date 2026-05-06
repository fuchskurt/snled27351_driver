//! I2C transport implementation for the SNLED27351 driver.
//!
//! Uses [`I2c`] with one 7-bit device address per driver chip.
//! Page selection and payload transfer are combined into a single
//! repeated-START transaction (Sr), which atomically holds the bus
//! across both operations and prevents another master or async task
//! from interleaving between page select and the actual transfer.
use crate::{
    registers::{CONFIGURE_CMD_PAGE, PWM_REGISTER_COUNT},
    transport::Transport,
};
use embedded_hal_async::i2c::{I2c, Operation};

/// I2C transport for `N` SNLED27351 driver chips on a shared bus.
///
/// All chips share one [`I2c`] bus instance and are distinguished by their
/// 7-bit addresses in `addrs`.
pub struct Controller<B, const N: usize>
where
    B: I2c,
{
    /// 7-bit I2C addresses, one per driver chip, in driver-index order.
    address: [u8; N],
    /// The I2C bus peripheral.
    bus:     B,
}

impl<B, const N: usize> Controller<B, N>
where
    B: I2c,
{
    /// Creates a new [`Controller`] from the given I2C bus and address list.
    ///
    /// `addrs` must be in the same order as the driver indices used by
    /// [`Driver`](crate::driver::Driver).
    #[inline]
    pub const fn new(bus: B, addrs: [u8; N]) -> Self { Self { address: addrs, bus } }
}

/// Errors that can occur in the I2C transport.
#[derive(Debug)]
#[non_exhaustive]
pub enum TransportError<E> {
    /// An I2C bus error occurred.
    I2c(E),
    /// The driver index was out of range.
    InvalidIndex,
    /// The data payload exceeded the maximum transfer size.
    PayloadTooLarge,
}

impl<B, const N: usize> Transport for Controller<B, N>
where
    B: I2c,
{
    type Error = TransportError<B::Error>;

    #[inline]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error> {
        let address = match self.address.get(driver_index) {
            Some(&addr) => addr,
            None => return Err(TransportError::InvalidIndex),
        };

        let mut buf = [0_u8; 1];

        // Page-select, register-pointer write, and read are one atomic Sr transaction.
        match self
            .bus
            .transaction(address, &mut [
                Operation::Write(&[CONFIGURE_CMD_PAGE, page]),
                Operation::Write(&[reg]),
                Operation::Read(&mut buf),
            ])
            .await
        {
            Ok(()) => {},
            Err(error) => return Err(TransportError::I2c(error)),
        }

        match buf.first() {
            Some(&val) => Ok(val),
            None => Err(TransportError::InvalidIndex),
        }
    }

    /// I2C chips are reset via the software shutdown register in
    /// [`crate::driver::Driver::init`]; no hardware pin toggle is needed here.
    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> { Ok(()) }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let address = match self.address.get(driver_index) {
            Some(&addr) => addr,
            None => return Err(TransportError::InvalidIndex),
        };

        // Build the outgoing frame: [reg_addr, payload...] into a fixed buffer.
        let mut buf = [0; PWM_REGISTER_COUNT.saturating_add(1)];

        match buf.get_mut(0) {
            Some(slot) => *slot = reg,
            None => return Err(TransportError::InvalidIndex),
        }

        let payload_end = match data.len().checked_add(1) {
            Some(end) => end,
            None => return Err(TransportError::PayloadTooLarge),
        };

        match buf.get_mut(1..payload_end) {
            Some(slot) => slot.copy_from_slice(data),
            None => return Err(TransportError::PayloadTooLarge),
        }

        let out = match buf.get(..payload_end) {
            Some(slice) => slice,
            None => return Err(TransportError::PayloadTooLarge),
        };

        // Issue page-select and data write as one atomic Sr transaction so no
        // other master or task can interleave between the two frames.
        match self
            .bus
            .transaction(address, &mut [Operation::Write(&[CONFIGURE_CMD_PAGE, page]), Operation::Write(out)])
            .await
        {
            Ok(()) => Ok(()),
            Err(error) => Err(TransportError::I2c(error)),
        }
    }
}
