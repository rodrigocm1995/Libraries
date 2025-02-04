/**
    *******************************************************************************************
  * @file           : HDC1080.h
  * @brief          : HDC1080 Library
    *******************************************************************************************

  * The HDC1080 is a digital humidity sensor with integrated temperature sensor that provides 
  * excellent measurement accuracy at very low power. The HDC1080 operates over a wide supply 
  * range, and is a low cost, low power alternative to competitive solutions in a wide range 
  * of common applications.
  *   
  * @details
  * Relative Humidity Accurary ±2% (typical)
  * Temperature accurary ±0.2°C (typical)
  * Excellent Stability and at High Humidity
  * 14 bit measurement resolution
  * 100 nA sleep mode current
  * Supply Voltage 2.7V to 5.5V
  * I2C interface
  * @example
  * 
  *******************************************************************************************
  */
#ifndef INC_HDC1080_H_
#define INC_HDC1080_H_

#define HDC1080_ADDRESS                  0x40
#define HDC1080_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define HDC1080_TEMPERATURE_REGISTER             0x00 // Temperature measurement output
#define HDC1080_HUMIDITY_REGISTER                0x01 // Relative Humidity measurement output
#define HDC1080_CONFIGURATION_REGISTER           0x02 // HDC1080 configuration and status
#define HDC1080_SERIAL_ID1_REGISTER              0xFB // First 2 bytes of the serial ID of the part
#define HDC1080_SERIAL_ID2_REGISTER              0xFC // Mid 2 bytes of the serial ID of the part
#define HDC1080_SERIAL_ID3_REGISTER              0xFD // Last byte of the serial ID of the part 
#define HDC1080_MANUFACTURER_ID_REGISTER         0xFE // ID of Texas Instruments (TI) = 0x5449
#define HDC1080_DEVICE_ID_REGISTER               0xFF // ID of the device = 0x1050

typedef enum
{
  HDC1080_HUMIDITY_14BIT_RESOLUTION           = 0x0000,
  HDC1080_HUMIDITY_11BIT_RESOLUTION           = 0x0100,
  HDC1080_HUMIDITY_8BIT_RESOLUTION            = 0x0200,
}HDC1080HumidityResolution_t;

typedef enum
{
  HDC1080_TEMP_14BIT_RESOLUTION               = 0x0000,
  HDC1080_TEMP_11BIT_RESOLUTION               = 0x0400,

}HDC1080TempResolution_t;

typedef enum
{
  HDC1080_BAT_VOLTAGE_G2_8                    = 0x0000,
  HDC1080_BAT_VOLTAGE_L2_8                    = 0x0800,
} HDC1080BatteryStatus_t;

typedef enum
{
  HDC1080_TEMP_OR_HUMIDITY                    = 0x0000,
  HDC1080_TEMP_AND_HUMIDITY                   = 0x1000,
} HDC1080BAcquisitionMode_t;

typedef enum
{
  HDC1080_HEATER_ENABLED                      = 0x2000,
  HDC1080_HEATER_DISABLED                     = 0x0000,
} HDC1080HeaterMode_t;


class HDC1080{
  public:
    HDC1080(); // Constructor. It must be defined with the same name as the class
    uint8_t  init(uint8_t devAddress, TwoWire *wire = &Wire);
    uint8_t  customInit(uint8_t devAddress, HDC1080HeaterMode_t heater, HDC1080BAcquisitionMode_t mode, HDC1080BatteryStatus_t batStatus, HDC1080TempResolution_t tempRes, HDC1080HumidityResolution_t humRes, TwoWire *wire = &Wire);
    uint8_t writeRegister(uint8_t registerAddress, uint16_t value);
    uint16_t readRegister(uint8_t registerAddress);
    uint16_t getConfig();
    uint16_t getTemperature();
    uint16_t getHumidity();
  
  private:
    uint8_t _deviceAddress = HDC1080_ADDRESS;
    TwoWire * _wire; //The generic connection to user's chosen I2C hardware
};

#endif