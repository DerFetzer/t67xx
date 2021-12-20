#![doc = include_str!("../README.md")]
#![no_std]
#![forbid(unsafe_code)]

use device_driver::ll::register::RegisterInterface;
use device_driver::{create_low_level_device, implement_registers, Bit};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write};
use num_enum::{IntoPrimitive, TryFromPrimitive};

/// Default I2C address of the sensor
pub const DEFAULT_ADDRESS: u8 = 0x15;

// Constants to make the register definitions more readable
const RA_FW: RegisterAddress = RegisterAddress {
    address: 0x1389,
    read_code: Some(ModbusFunctionCode::ReadInternalRegisters),
    write_code: None,
};
const RA_STATUS: RegisterAddress = RegisterAddress {
    address: 0x138A,
    read_code: Some(ModbusFunctionCode::ReadInternalRegisters),
    write_code: None,
};
const RA_GAS_PPM: RegisterAddress = RegisterAddress {
    address: 0x138B,
    read_code: Some(ModbusFunctionCode::ReadInternalRegisters),
    write_code: None,
};
const RA_RESET: RegisterAddress = RegisterAddress {
    address: 0x03E8,
    read_code: None,
    write_code: Some(ModbusFunctionCode::ForceSingleCoil),
};
const RA_SPC: RegisterAddress = RegisterAddress {
    address: 0x03EC,
    read_code: None,
    write_code: Some(ModbusFunctionCode::ForceSingleCoil),
};
const RA_SLAVE_ADDR: RegisterAddress = RegisterAddress {
    address: 0x0FA5,
    read_code: Some(ModbusFunctionCode::ReadHoldingRegisters),
    write_code: Some(ModbusFunctionCode::PresetSingleRegister),
};
const RA_ABC: RegisterAddress = RegisterAddress {
    address: 0x03EE,
    read_code: None, // Some(ModbusFunctionCode::ReadCoilStatus),
    write_code: Some(ModbusFunctionCode::ForceSingleCoil),
};

/// Modbus function codes
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum ModbusFunctionCode {
    /// Read output coils
    ReadCoilStatus = 0x01,
    /// Read holding registers
    ReadHoldingRegisters = 0x03,
    /// Read internal registers
    ReadInternalRegisters = 0x04,
    /// Write single coil
    ForceSingleCoil = 0x05,
    /// Write single register
    PresetSingleRegister = 0x06,
}

/// Encapsulates the address of the register as well as the function codes for reading and writing
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct RegisterAddress {
    /// Register address
    pub address: u16,
    /// Function code for reading
    pub read_code: Option<ModbusFunctionCode>,
    /// Function code for writing
    pub write_code: Option<ModbusFunctionCode>,
}

/// Valid values for ABC-Logic
#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum AbcLogic {
    /// Enable
    Enabled = 0xFF00,
    /// Disable
    Disabled = 0x0000,
}

/// Valid values for single point calibration
#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum SinglePointCalibration {
    /// Start
    Start = 0xFF00,
    /// Stop
    Stop = 0x0000,
}

/// Valid values for Reset
#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum Reset {
    /// Reset
    Reset = 0xFF00,
}

/// The errors our hardware interface can return.
#[derive(Debug)]
pub enum InterfaceError {
    /// I2C error
    CommunicationError,
    /// Invalid command for reading/writing
    InvalidCommand,
    /// Error response from sensor
    ModbusError(u8),
}

/// Our full hardware interface with the chip
pub struct T67xxInterface<I2C: Read + Write, D: DelayMs<u8>> {
    /// The I2C interface we use to drive the sensor
    communication_interface: I2C,
    /// Some kind of delay provider
    delay: D,
    /// I2C address of the chip
    addr: u8,
}

impl<I2C: Read + Write, D: DelayMs<u8>> T67xxInterface<I2C, D> {
    /// Creates a new hardware interface
    ///
    /// If the address is `None` [DEFAULT_ADDRESS] is used.
    pub fn new(communication_interface: I2C, delay: D, address: Option<u8>) -> Self {
        T67xxInterface {
            communication_interface,
            delay,
            addr: address.unwrap_or(DEFAULT_ADDRESS),
        }
    }

    /// Frees the peripherals
    pub fn free(self) -> (I2C, D) {
        (self.communication_interface, self.delay)
    }
}

impl<I2C: Read + Write, D: DelayMs<u8>> RegisterInterface for T67xxInterface<I2C, D> {
    type Address = RegisterAddress;
    type InterfaceError = InterfaceError;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        if value.len() != 2 {
            return Err(InterfaceError::InvalidCommand);
        }
        let mut buf: [u8; 4] = [0; 4];
        self.communication_interface
            .write(
                self.addr,
                &[
                    address.read_code.ok_or(InterfaceError::InvalidCommand)? as u8,
                    (address.address >> 8) as u8,
                    (address.address & 0xff) as u8,
                    0x00,
                    0x01,
                ],
            )
            .map_err(|_| InterfaceError::CommunicationError)?;
        // Delay according to datasheet
        self.delay.delay_ms(10);
        // Read response and check function code
        self.communication_interface
            .read(self.addr, &mut buf[..])
            .map_err(|_| InterfaceError::CommunicationError)?;
        if buf[0] != address.read_code.unwrap() as u8 {
            return Err(InterfaceError::ModbusError(buf[0]));
        }
        value.copy_from_slice(&buf[2..]);
        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), Self::InterfaceError> {
        if value.len() != 2 {
            return Err(InterfaceError::InvalidCommand);
        }
        let mut buf = [
            address.write_code.ok_or(InterfaceError::InvalidCommand)? as u8,
            (address.address >> 8) as u8,
            (address.address & 0xff) as u8,
            0,
            0,
        ];
        buf[3..].copy_from_slice(value);
        self.communication_interface
            .write(self.addr, &buf)
            .map_err(|_| InterfaceError::CommunicationError)?;
        // Delay according to datasheet
        self.delay.delay_ms(10);
        if address != RA_RESET {
            // Read response and check function code
            self.communication_interface
                .read(self.addr, &mut buf[..])
                .map_err(|_| InterfaceError::CommunicationError)?;
            if buf[0] != address.write_code.unwrap() as u8 {
                return Err(InterfaceError::ModbusError(buf[0]));
            }
        }
        Ok(())
    }
}

impl<I2C: Read + Write, D: DelayMs<u8>> HardwareInterface for T67xxInterface<I2C, D> {}

// Create our low level device. This holds all the hardware communication definitions
create_low_level_device!(
    /// Low level access to the T67xx chip
    T67xxLL {
        // The types of errors our low level error enum must contain
        errors: [InterfaceError],
        hardware_interface_requirements: { RegisterInterface<Address = RegisterAddress, InterfaceError = InterfaceError> },
        hardware_interface_capabilities: {}
    }
);

// Create a register set for the device
implement_registers!(
    /// The global register set
    T67xxLL.registers<RegisterAddress> = {
        /// Returns the firmware revision from the sensor
        #[generate(Debug)]
        firmware_revision(RO, RA_FW, 2) = {
            /// Firmware revision
            firmware_revision: u16:BE = RO 0..16,
        },
        /// Returns a status register from the sensor
        #[generate(Debug)]
        status(RO, RA_STATUS, 2) = MSB {
            /// Error condition
            error_condition: u8 as Bit = RO 15..=15,
            /// Flash error
            flash_error: u8 as Bit = RO 14..=14,
            /// Calibration error
            calibration_error: u8 as Bit = RO 13..=13,
            /// RS-232
            rs232: u8 as Bit = RO 7..=7,
            /// RS-485
            rs485: u8 as Bit = RO 6..=6,
            /// I2C
            i2c: u8 as Bit = RO 5..=5,
            /// Warm-up mode
            warm_up_mode: u8 as Bit = RO 4..=4,
            /// Single point calibration
            single_point_calibration: u8 as Bit = RO 0..=0,
        },
        /// The current gas ppm calculation
        #[generate(Debug)]
        gas_ppm(RO, RA_GAS_PPM, 2) = {
            /// Gas ppm
            gas_ppm: u16:BE = RO 0..16,
        },
        /// Reset the sensor over the Modbus network
        reset_device(WO, RA_RESET, 2) = {
            /// Reset code
            reset_device: u16:BE as Reset = WO 0..16,
        },
        /// Single-point calibration
        start_single_point_cal(WO, RA_SPC, 2) = {
            /// Enable/disable single-point calibration
            start_single_point_cal: u16:BE as SinglePointCalibration = WO 0..16,
        },
        /// Change of sensor address (default address is 0x15)
        #[generate(Debug)]
        slave_address(RW, RA_SLAVE_ADDR, 2) = {
            /// Sensor address
            slave_address: u8 = RW 8..16,
        },
        /// Enable or disable ABC Logic™
        abc_logic(WO, RA_ABC, 2) = {
            /// Enable/disable ABC Logic™
            abc_logic: u16:BE as AbcLogic = WO 0..16,
        },
    }
);
