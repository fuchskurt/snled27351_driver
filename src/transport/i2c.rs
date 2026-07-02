//! I2C transport implementation for the SNLED27351 driver.
//!
//! Uses [`I2c`] with one 7-bit device address per driver chip.
//!
//! # Framing
//!
//! The SNLED27351 treats every I2C write frame as `[register, data...]`,
//! so each register access is two bus frames:
//!
//! 1. A write to the command register (`0xFD`) selecting the active page.
//! 2. The register write (or a write-read for register reads).
//!
//! The page select must be its own STOP-terminated frame: it cannot be
//! combined with the following access inside one `transaction()`, because
//! [`I2c::transaction`] merges adjacent write operations into a single
//! frame — the chip would then see `[0xFD, page, reg, data...]` and
//! auto-increment the payload into the wrong registers.
//!
//! Splitting into two frames is still race-free for this driver: it holds
//! `&mut` access to the bus (or bus device) for the whole call, and traffic
//! to other devices on a shared bus does not affect this chip's page state.
use crate::{
    registers::{CONFIGURE_CMD_PAGE, PWM_REGISTER_COUNT},
    transport::Transport,
};
use embedded_hal_async::i2c::I2c;

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

    /// Selects the active register page on the chip at `address` by writing
    /// the page number to the command register (`0xFD`).
    ///
    /// See the module documentation for why this is a separate bus frame.
    async fn select_page(&mut self, address: u8, page: u8) -> Result<(), TransportError<B::Error>> {
        self.bus.write(address, &[CONFIGURE_CMD_PAGE, page]).await.map_err(TransportError::I2c)
    }
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
        let Some(&address) = self.address.get(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };

        self.select_page(address, page).await?;

        // Write the register pointer, then read one byte after a repeated
        // START.
        let mut buf = [0_u8; 1];
        self.bus.write_read(address, &[reg], &mut buf).await.map_err(TransportError::I2c)?;

        let [value] = buf;
        Ok(value)
    }

    /// I2C chips are reset via the software shutdown register in
    /// [`crate::driver::Driver::init`]; no hardware pin toggle is needed here.
    #[expect(
        clippy::unused_async_trait_impl,
        reason = "reset is a hardware no-op on I2C; the trait method must stay async for the SPI implementation"
    )]
    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> { Ok(()) }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let Some(&address) = self.address.get(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };

        self.select_page(address, page).await?;

        // Build the outgoing frame [reg, payload...] in a fixed buffer.
        let mut buf = [0_u8; PWM_REGISTER_COUNT.saturating_add(1)];
        let frame_len = data.len().saturating_add(1);
        let Some(frame) = buf.get_mut(..frame_len) else {
            return Err(TransportError::PayloadTooLarge);
        };
        let Some((reg_slot, payload)) = frame.split_first_mut() else {
            return Err(TransportError::PayloadTooLarge);
        };
        *reg_slot = reg;
        payload.copy_from_slice(data);

        self.bus.write(address, frame).await.map_err(TransportError::I2c)
    }
}
