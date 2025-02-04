/**
    *******************************************************************************************
  * @file           : TPL1401.h
  * @brief          : TPL1401 Library
    *******************************************************************************************

  * The TPL1401 is a digital potentiometer (digipot) with a buffered wiper. Unlike standard di-
  * gipots, this device offers higher load regulation in voltage-divider applications as a res-
  * ult of the integrated buffered wiper.
  * 
  * The TPL1401 makes in-factory calibration and trimming easier with integrated nonvolatile me
  * mory (NVM), and a simple I²C digital interface to communicate with the device. This device 
  * supports I²C standard mode (100 kbps), fast mode (400 kbps), and fast mode plus (1 Mbps).
  * 
  * The TPL1401 operates with either the internal reference or with the power supply as the re-
  * ference and provides a full-scale output of 1.8V to 5.5V. This device also includes a wiper
  * lock feature, a feedback (FB) pin for current-sink applications, and two bytes of user pro-
  * grammable NVM space. The TPL1401 has a power-on-reset (POR) circuit that makes sure all the 
  * registers start with default or user-programmed settings using NVM. The digipot output po-
  * wers on in high-impedance mode (default); this settings can be programmed to 10kΩ-GND using
  * NVM.
  *   
  * @details
  * 256-position digital potentiometer for voltage-divider applications
  * 1 LSB INL and DNL
  * Wide operating range.
  * - Power supply: 1.8V to 5.5V
  * -Temperature range: -40°C to +125°C
  * Buffered wiper for improved load regulation.
  * FB pin for precision current sink applications.
  * Wiper lock function to protect from accidental writes to the digital potentiometer
  * I²C interface
  * - Standard, fast, and fast plus modes
  * - 1.62V V_IH with V_DD = 5.5V
  * User programmable nonvolatile memory (NVM/EEPROM)
  * - Save and recall all register  settings
  * Internal Reference
  * Very low power: 0.2mA at 1.8V
  * Flexible startup: High impedance or 10kΩ-GND
  * Tiny package: 8-pin WSON (2mm x 2mm)
  * @example
  * 
  *******************************************************************************************
  */
#ifndef INC_TPL1401_H_
#define INC_TPL1401_H_

#include <Arduino.h>
#include <Wire.h>

// MACRO TO CHECK IF A CERTAIN BIT IS SET
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define TPL1401_DEFAULT_ADDRESS                     0x49
#define TPL1401_TRIALS                              5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define TPL1401_STATUS_REGISTER_REGISTER            0xD0  // Read only
#define TPL1401_GENERAL_CONFIG_REGISTER             0xD1
#define TPL1401_PROTECT_REGISTER                    0xD3
#define TPL1401_DPOT_POSITION_REGISTER              0x21
#define TPL1401_USER_BYTE1_REGISTER                 0x25
#define TPL1401_USER_BYTE2_REGISTER                 0x26

#define TPL1401_NVM_CRC_ALARM_USER                  0x8000  // 0: No CRC error in user NVM bits → 1: CRC error in user NVM bits
#define TPL1401_NVM_CRC_ALARM_INTERNAL              0x4000  // 0: No CRC error in internal NVM  → 1: CRC error in internal NVM bits
#define TPL1401_NVM_BUSY                            0x2000  // 0: NVM write or load completed, write to digipot registers allowed  → 1: NVM write or load in progress, write to digipot registers not allowed  

// Config Bits
#define TPL1401_DEVICE_LOCK_BIT                     0x2000
#define TPL1401_RESERVED_BITS                       0x01E0
#define TPL1401_REF_ENABLED                         0x0004

typedef enum
{
  POWER_UP                                        = 0x0000,
  POWER_DOWN_TO_10K                               = 0x0008,
  POWER_DOWN_HIGH_IMPEDANCE                       = 0x0010,
} Tpl1401DpotPdn_t;

typedef enum
{
  VOUT_GAIN_1_5X                                  = 0x0000,
  VOUT_GAIN_2X                                    = 0x0001,
  VOUT_GAIN_3X                                    = 0x0002,
  VOUT_GAIN_4X                                    = 0x0003
} Tpl1401OutSpan_t;

class TPL1401{
  public:
    TPL1401(); // Constructor. It must be defined with the same name as the class
    uint8_t  init(uint8_t devAddress, TwoWire *wire = &Wire);
    uint8_t  customInit(uint8_t devAddress, uint16_t deviceLock, Tpl1401DpotPdn_t powerType, uint16_t internalRef, Tpl1401OutSpan_t span, TwoWire *wire = &Wire);
    uint8_t writeRegister(uint8_t registerAddress, uint16_t value);
    uint16_t readRegister(uint8_t registerAddress);
    uint16_t getConfig();
  
  private:
    uint8_t _deviceAddress = TPL1401_DEFAULT_ADDRESS;
    TwoWire * _wire; //The generic connection to user's chosen I2C hardware
};

#endif