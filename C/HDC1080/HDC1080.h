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
  * 
  * 
  *******************************************************************************************
  */

#ifndef INC_HDC1080_H_
#define INC_HDC1080_H_

#define HDC1080_ADDRESS                         0x40
#define HDC1080_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define HDC1080_TEMPERATURE_REG                 0x00 /* Temperature measurement output */ 
#define HDC1080_HUMIDITY_REG                    0x01 /* Relative Humidity measurement output */ 
#define HDC1080_CONFIGURATION_REG               0x02 /* HDC1080 configuration and status */ 
#define HDC1080_SERIAL_ID1_REG                  0xFB /* First 2 bytes of the serial ID of the part */ 
#define HDC1080_SERIAL_ID2_REG                  0xFC /* Mid 2 bytes of the serial ID of the part */ 
#define HDC1080_SERIAL_ID3_REG                  0xFD /* Last byte of the serial ID of the part  */ 
#define HDC1080_MANUFACTURER_ID_REG             0xFE /* ID of Texas Instruments (TI) = 0x5449 */ 
#define HDC1080_DEVICE_ID_REG                   0xFF /* ID of the device = 0x1050 */ 

/*******************  Bits definition for CONFIGURATION register  ******************/
#define HDC1080_HRES_Pos                        (8U)
#define HDC1080_HRES_Mask                       (0x3U << HDC1080_HRES_Pos)
#define HDC1080_HRES                            HDC1080_HRES_Mask

#define HDC1080_TRES_Pos                        (10U)
#define HDC1080_TRES_Mask                       (0x1U << HDC1080_TRES_Pos)
#define HDC1080_TRES                            HDC1080_TRES_Mask

#define HDC1080_BTST_Pos                        (11U)
#define HDC1080_BTST_Mask                       (0x1U << HDC1080_BTST_Pos)
#define HDC1080_BTST                            HDC1080_BTST_Mask

#define HDC1080_MODE_Pos                        (12U)
#define HDC1080_MODE_Mask                       (0x1U << HDC1080_MODE_Pos)
#define HDC1080_MODE                            HDC1080_MODE_Mask

#define HDC1080_HEAT_Pos                        (13U)
#define HDC1080_HEAT_Mask                       (0x1U << HDC1080_HEAT_Pos)
#define HDC1080_HEAT                            HDC1080_HEAT_Mask

#define HDC1080_RST_Pos                         (15U)
#define HDC1080_RST_Mask                        (0x1U << HDC1080_RST_Pos)
#define HDC1080_RST                             HDC1080_RST_Mask

typedef enum
{
  HDC1080_14BIT_RESOLUTION                      = 0x0U,
  HDC1080_11BIT_RESOLUTION                      = 0x1U,
  HDC1080_8BIT_RESOLUTION                       = 0x2U,
} HDC1080_Resolution_TypeDef;

typedef enum
{
  HDC1080_BAT_VOLTAGE_G2_8                    = 0x0000,
  HDC1080_BAT_VOLTAGE_L2_8                    = 0x0800,
} HDC1080BatteryStatus_t;

typedef enum
{
  HDC1080_TEMP_OR_HUMIDITY                    = 0x0U,
  HDC1080_TEMP_AND_HUMIDITY                   = 0x1U,
} HDC1080_Mode_TypeDef;

typedef enum
{
  HDC1080_HEATER_DISABLED                     = 0x0U,
  HDC1080_HEATER_ENABLED                      = 0x1U,
} HDC1080_Heater_TypeDef;

typedef struct
{
  I2C_HandleTypeDef         *hi2c;
  uint8_t                   _devAddress;
} HDC1080_HandleTypeDef;


/* Register Write & Read */
HAL_StatusTypeDef HDC1080_WriteRegister(HDC1080_HandleTypeDef *hdc1080, uint8_t registerAddress, uint16_t value);
uint16_t HDC1080_ReadRegister(HDC1080_HandleTypeDef *hdc1080, uint8_t registerAddress);

/* Initialization & Control */
void HDC1080_Init(HDC1080_HandleTypeDef *hdc1080, I2C_HandleTypeDef *i2c, uint8_t devAddress);
void HDC1080_ResetDevice(HDC1080_HandleTypeDef *hdc1080);


void HDC1080_SetHumResolution(HDC1080_HandleTypeDef *hdc1080, HDC1080_Resolution_TypeDef resolution);
void HDC1080_SetTempResolution(HDC1080_HandleTypeDef *hdc1080, HDC1080_Resolution_TypeDef resolution);
void HDC1080_SetMode(HDC1080_HandleTypeDef *hdc1080, HDC1080_Mode_TypeDef mode);
void HDC1080_SetHeater(HDC1080_HandleTypeDef *hdc1080, HDC1080_Heater_TypeDef heat);


double HDC1080_GetTemp_C(HDC1080_HandleTypeDef *hdc1080);

double HDC1080_GetHumidity(HDC1080_HandleTypeDef *hdc1080);

#endif
