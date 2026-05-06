//! SNLED27351 LED driver.
//!
//! Generic over a [`Transport`] `T` so the same driver logic covers both
//! SPI and I2C without any duplication. `N` is the number of driver chips
//! and must match the number of devices managed by the transport.
use crate::{
    registers::{
        CURRENT_TUNE_REGISTER_COUNT,
        LED_CONTROL_REGISTER_COUNT,
        MSK_THERMAL_FLAG,
        PAGE_CURRENT_TUNE,
        PAGE_FUNCTION,
        PAGE_LED_CONTROL,
        PAGE_PWM,
        PULLDOWNUP_ALL_ENABLED,
        PWM_REGISTER_COUNT,
        REG_PULLDOWNUP,
        REG_SCAN_PHASE,
        REG_SLEW_RATE_CONTROL_MODE_1,
        REG_SLEW_RATE_CONTROL_MODE_2,
        REG_SOFTWARE_SHUTDOWN,
        REG_SOFTWARE_SLEEP,
        REG_THERMAL,
        SCAN_PHASE_12_CHANNEL,
        SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE,
        SLEW_RATE_CONTROL_MODE_2_ALL_ENABLE,
        SOFTWARE_SHUTDOWN_SSD_NORMAL,
        SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
        SOFTWARE_SLEEP_DISABLE,
    },
    transport::Transport,
};

/// Mapping of a single RGB LED to its three PWM register addresses on one
/// driver chip.
#[derive(Copy, Clone)]
#[non_exhaustive]
pub struct Led {
    /// PWM register address for the blue channel.
    blue:   u8,
    /// Index of the driver chip controlling this LED.
    driver: usize,
    /// PWM register address for the green channel.
    green:  u8,
    /// PWM register address for the red channel.
    red:    u8,
}

impl Led {
    /// Returns the PWM register address for the blue channel.
    #[inline]
    #[must_use]
    pub const fn blue(self) -> u8 { self.blue }

    /// Returns the index of the driver chip controlling this LED.
    ///
    /// The index corresponds to the position in the transport's device array
    /// (e.g. the `addrs` slice for I2C or the `devices` array for SPI).
    #[inline]
    #[must_use]
    pub const fn driver(self) -> usize { self.driver }

    /// Returns the PWM register address for the green channel.
    #[inline]
    #[must_use]
    pub const fn green(self) -> u8 { self.green }

    /// Creates a new [`Led`] with the given driver index and channel register
    /// addresses.
    ///
    /// `driver` is the zero-based index of the chip that controls this LED,
    /// matching the order of devices passed to the transport constructor.
    /// `red`, `green`, and `blue` are the raw PWM register addresses on that
    /// chip, as defined by the `CB*_CA*` constants in the register map.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use snled27351_driver::driver::Led;
    /// use snled27351_driver::registers::{CB1_CA1, CB2_CA1, CB3_CA1};
    ///
    /// const MY_LED: Led = Led::new(0, CB1_CA1, CB3_CA1, CB2_CA1);
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(driver: usize, red: u8, green: u8, blue: u8) -> Self { Self { blue, driver, green, red } }

    /// Returns the PWM register address for the red channel.
    #[inline]
    #[must_use]
    pub const fn red(self) -> u8 { self.red }
}

/// PWM shadow buffer and dirty flag for a single driver chip.
struct DriverBuf {
    /// Whether this driver's buffer has unsent changes.
    dirty: bool,
    /// PWM shadow buffer; index is the raw register address.
    pwm:   [u8; PWM_REGISTER_COUNT],
}

impl DriverBuf {
    /// Creates a new zeroed, clean [`DriverBuf`].
    const fn new() -> Self { Self { dirty: false, pwm: [0_u8; PWM_REGISTER_COUNT] } }

    /// Writes RGB values into the shadow buffer for one LED and marks dirty.
    const fn stage(&mut self, led: Led, red: u8, green: u8, blue: u8) {
        if let Some(slot) = self.pwm.get_mut(usize::from(led.red)) {
            *slot = red;
        }
        if let Some(slot) = self.pwm.get_mut(usize::from(led.green)) {
            *slot = green;
        }
        if let Some(slot) = self.pwm.get_mut(usize::from(led.blue)) {
            *slot = blue;
        }
        self.dirty = true;
    }
}

/// SNLED27351 LED driver.
///
/// Generic over a [`Transport`] `T` so the same driver logic covers both
/// SPI and I2C without any duplication. `N` is the number of driver chips
/// and must match the number of devices managed by the transport.
///
/// # Usage
///
/// ```ignore
/// let transport = SpiTransport::new(devices, sdb_pin);
/// let mut driver = Driver::new(transport, &LED_LAYOUT);
/// driver.init(0xFF).await?;
/// driver.set_all_leds(255, 255, 255).await?;
/// ```
pub struct Driver<T, const N: usize>
where
    T: Transport,
{
    /// PWM shadow buffers, one per chip.
    bufs:      [DriverBuf; N],
    /// LED layout mapping logical indices to driver channels.
    leds:      &'static [Led],
    /// The underlying bus transport.
    transport: T,
}

impl<T, const N: usize> Driver<T, N>
where
    T: Transport,
{
    /// Reads the thermal detector flag (TDF) from driver chip `index`.
    ///
    /// Returns `true` if the chip reports temperature >= 70 °C.
    /// Returns `false` if the index is out of range or the read fails.
    #[inline]
    pub async fn check_thermal_flag_set(&mut self, index: usize) -> bool {
        self.transport
            .read_reg(index, PAGE_FUNCTION, REG_THERMAL)
            .await
            .is_ok_and(|thermal_reg| thermal_reg & MSK_THERMAL_FLAG != 0)
    }

    /// Flushes all dirty PWM shadow buffers to hardware.
    ///
    /// Only drivers with pending changes are written, so calling this after
    /// a no-op `stage_led` is free.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if a bus transaction fails while writing a
    /// dirty buffer. Buffers that were successfully flushed before the error
    /// are marked clean; the failed buffer remains dirty and will be retried
    /// on the next call.
    #[inline]
    pub async fn flush(&mut self) -> Result<(), T::Error> {
        for chip_index in 0..N {
            let dirty = self.bufs.get(chip_index).is_some_and(|buf| buf.dirty);
            if !dirty {
                continue;
            }
            // Copy the PWM buffer to the stack to release the borrow on
            // `self.bufs` before the async transport call.
            let pwm = {
                let Some(buf) = self.bufs.get(chip_index) else { continue };
                buf.pwm
            };
            match self.transport.write_page(chip_index, PAGE_PWM, 0x00, &pwm).await {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            if let Some(buf) = self.bufs.get_mut(chip_index) {
                buf.dirty = false;
            }
        }
        Ok(())
    }

    /// Initializes all `N` driver chips.
    ///
    /// Performs the hardware reset sequence via the transport, then for each
    /// chip: configures the function page registers, clears all LEDs and PWM
    /// values, programs current tune, enables all channels, and releases
    /// shutdown.
    ///
    /// `current_tune` sets the constant current step for all CB channels
    /// (0xFF = 40 mA at the default register scaling).
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails during
    /// initialization. The chip state is undefined if this occurs.
    #[inline]
    pub async fn init(&mut self, current_tune: u8) -> Result<(), T::Error> {
        match self.transport.reset().await {
            Ok(()) => {},
            Err(err) => return Err(err),
        }

        for chip_index in 0..N {
            // Enter shutdown so registers can be safely programmed.
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_SHUTDOWN])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }

            // Configure function registers.
            match self.transport.write_page(chip_index, PAGE_FUNCTION, REG_PULLDOWNUP, &[PULLDOWNUP_ALL_ENABLED]).await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            match self.transport.write_page(chip_index, PAGE_FUNCTION, REG_SCAN_PHASE, &[SCAN_PHASE_12_CHANNEL]).await {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, &[
                    SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE,
                ])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, &[
                    SLEW_RATE_CONTROL_MODE_2_ALL_ENABLE,
                ])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, &[SOFTWARE_SLEEP_DISABLE])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }

            // Clear LED control registers (all off).
            match self
                .transport
                .write_page(chip_index, PAGE_LED_CONTROL, 0x00, &[0x00_u8; LED_CONTROL_REGISTER_COUNT])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }

            // Zero all PWM registers and sync the shadow buffer.
            match self.transport.write_page(chip_index, PAGE_PWM, 0x00, &[0x00_u8; PWM_REGISTER_COUNT]).await {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
            if let Some(buf) = self.bufs.get_mut(chip_index) {
                buf.pwm.fill(0x00);
                buf.dirty = false;
            }

            // Program current tune for all CB channels.
            match self
                .transport
                .write_page(chip_index, PAGE_CURRENT_TUNE, 0x00, &[current_tune; CURRENT_TUNE_REGISTER_COUNT])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }

            // Enable all LED channels.
            match self
                .transport
                .write_page(chip_index, PAGE_LED_CONTROL, 0x00, &[0xFF_u8; LED_CONTROL_REGISTER_COUNT])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }

            // Release shutdown — chip enters normal operating mode.
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_NORMAL])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
        }

        Ok(())
    }

    /// Creates a new driver from the given transport and LED layout.
    ///
    /// Call [`init`](Self::init) before writing any LED values.
    #[inline]
    pub const fn new(transport: T, leds: &'static [Led]) -> Self {
        Self { transport, leds, bufs: [const { DriverBuf::new() }; N] }
    }

    /// Writes pre-corrected PWM values for all LEDs and flushes immediately.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the flush fails. See [`flush`](Self::flush).
    #[inline]
    pub async fn set_all_leds(&mut self, red: u8, green: u8, blue: u8) -> Result<(), T::Error> {
        self.stage_all_leds(red, green, blue);
        self.flush().await
    }

    /// Updates the constant-current step for all CB channels on driver `index`.
    ///
    /// `value` maps to output current as `Iout = value × 0.157 mA` for
    /// values 4–255. Values 1–3 produce ~0.47 mA; 0 disables current.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the bus transaction fails.
    #[inline]
    pub async fn set_current_tune(&mut self, index: usize, value: u8) -> Result<(), T::Error> {
        self.transport.write_page(index, PAGE_CURRENT_TUNE, 0x00, &[value; CURRENT_TUNE_REGISTER_COUNT]).await
    }

    /// Writes pre-corrected PWM values for a single LED and flushes
    /// immediately.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the flush fails. See [`flush`](Self::flush).
    #[inline]
    pub async fn set_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) -> Result<(), T::Error> {
        self.stage_led(led_index, red, green, blue);
        self.flush().await
    }

    /// Puts all driver chips into hardware shutdown mode.
    ///
    /// Cuts LED current immediately. Call [`wake`](Self::wake) to resume.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn shutdown(&mut self) -> Result<(), T::Error> {
        for chip_index in 0..N {
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_SHUTDOWN])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }

    /// Stages RGB values for all LEDs in the layout without flushing.
    #[inline]
    pub fn stage_all_leds(&mut self, red: u8, green: u8, blue: u8) {
        let leds: &'static [Led] = self.leds;
        for &led in leds {
            let Some(buf) = self.bufs.get_mut(led.driver) else { continue };
            buf.stage(led, red, green, blue);
        }
    }

    /// Stages RGB values for one LED by layout index without flushing.
    ///
    /// Does nothing if `led_index` is out of bounds. Call
    /// [`flush`](Self::flush) to transmit pending changes.
    #[inline]
    pub const fn stage_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) {
        let Some(&led) = self.leds.get(led_index) else { return };
        let Some(buf) = self.bufs.get_mut(led.driver) else { return };
        buf.stage(led, red, green, blue);
    }

    /// Releases all driver chips from hardware shutdown mode.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn wake(&mut self) -> Result<(), T::Error> {
        for chip_index in 0..N {
            match self
                .transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_NORMAL])
                .await
            {
                Ok(()) => {},
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }
}
