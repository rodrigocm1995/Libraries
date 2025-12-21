/**
    *******************************************************************************************
  * @file           : MCP9808.h
  * @brief          : MCP9808 Library
    *******************************************************************************************

  * MCP9808 digital temperature sensor converts temperatures between -20°C and +100°C to a digital
  * word with ±0.25°C/±0.5°C (typical/maximum) accuracy.
  * 
  * The MCP9808 comes with user-programmable registers that provide flexibility for temperature
  * sensing applications. The registers allow user-selectable settings such as Shuntdown or Low-
  * Power modes and the specification of temperature alert window limits and critical output limits.
  * When the temperature changes beyond the specified boundary limits, the  MCP9808 outputs an
  * Alert signal. The user has the option of setting the Alert output signal polarity as an active-
  * low or active high comparator output for thermostat operation, or as a temperature Alert
  * interrupt output for microprocessor-based systems. The alert output can also be configured as
  * a criticaltemperature output only
  *
  *   
  * @details
  * Accuracy:
  * - ±0.25°C (typical) from -40°C to +125°C
  * - ±0.5°C (maximum) from -20°C to +100°C
  * - ±1°C (maximum) from from -40°C to +125°C
  * User selectable Measurement Resolution:
  * - +0.5°C, +0.25°C,m +0.125°C, +0.0625°C
  * User-Programmable Temperature Limits:
  * - Temperature Window Limit
  * - Critical Temperature Limit
  * User programmable Temperature Alert Output
  * Operating voltage range: 2.7V to 5.5V
  * Operating Current: 200µA (typical)
  * Shuntdown current: 0.1µA (typical)
  * 2-Wire interface: I2C/SMBus compatible
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_MCP9808_H_
#define INC_MCP9808_H_

#define MCP9808_ADDRESS                         0x18 //Selecting A0 = A1 = A2 = 0
#define MCP9808_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define MCP9808_CONFIG_REG                      0x01
#define MCP9808_ALERT_TEMP_UPPER_REG            0x02
#define MCP9808_ALERT_TEMP_LOWER_REG            0x03
#define MCP9808_CRITICAL_TEMP_REG               0x04
#define MCP9808_TEMPERATURE_REG                 0x05
#define MCP9808_MANUFACTURER_ID_REG             0x06 // 0x0054
#define MCP9808_DEVICE_ID_REG                   0x07
#define MCP9808_RESOLUTION_REG                  0x08


// Masks
#define MCP9808_THYST_MASK                      0xF9FF
#define MCP9808_SHDN_MASK                       0xFEFF
#define MCP9808_CRIT_LOCK_MASK                  0xFF7F
#define MCP9808_WIN_LOCK_MASK                   0xFFBF
#define MCP9808_INT_CLEAR_MASK					0xFFDF
#define MCP9808_ALERT_CTRL_MASK					0xFFF7
#define MCP9808_ALERT_POLARITY_MASK				0xFFFD

typedef enum
{
  MCP9808_HYST_0_CELSIUS                        = 0x0000,
  MCP9808_HYST_1_5_CELSIUS                      = 0x0200,
  MCP9808_HYST_3_0_CELSIUS                      = 0x0400,
  MCP9808_HYST_6_0_CELSIUS                      = 0x0600
}MCP9808_Hysteresis_t;


typedef enum
{
  MCP9808_CONTINUOUS_CONV                       = 0x0000,
  MCP9808_SHUNTDOWN                             = 0x0100,
}Mcp9808_shutdown;

typedef enum
{
  MCP9808_CRIT_UNLOCKED                         = 0x0000,
  MCP9808_CRIT_LOCKED                           = 0x0040,
} Mcp9808_crit_lock;

// T UPPER and TLOWER Window Lock bit
typedef enum
{
  MCP9808_WIN_UNLOCKED                          = 0x0000,
  MCP9808_WIN_LOCKED                            = 0x0080,
} Mcp9808_window_lock;

// Interrupt Clear bit
typedef enum
{
  MCP9808_CLEAR_INT_NO_EFFECT                   = 0x0000,
  MCP9808_CLEAR_INT_OUTPUT                      = 0x0020,
} MCP9808_IntClear_t;

typedef enum
{
  MCP9808_ALERT_CTRL_DISABLED                  = 0x0000,
  MCP9808_ALERT_CTRL_ENABLED                   = 0x0008,
} MCP9808_AlertCtrl_t;

typedef enum
{
	MCP9808_0_5C_RES							= 0x00,
	MCP9808_0_25C_RES							= 0x01,
	MCP9808_0_125C_RES							= 0x02,
	MCP9808_0_0625C_RES							= 0x03,
}MCP9808_Resolution_t;


typedef enum
{
	MCP9808_ALERT_ACTIVE_LOW					= 0x0002,
	MCP9808_ALERT_ACTIVE_HIGH					= 0x0000,
}MCP9808_AlertPolarity_t;
typedef enum
{
	MCP9808_MEMADD_SIZE_8BIT					= 0x01U,
	MCP9808_MEMADD_SIZE_16BIT					= 0x02U,
}MCP9808_AddressSize_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t           devAddress;
}MCP9808_HandleTypeDef;



HAL_StatusTypeDef MCP9808_WriteRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress, uint16_t value, MCP9808_AddressSize_t size);

uint16_t MCP9808_ReadRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress);

void MCP9808_Init(MCP9808_HandleTypeDef *mcp9808, I2C_HandleTypeDef *i2c);

uint16_t MCP9808_GetConfig(MCP9808_HandleTypeDef *mcp9808);


/**
    *******************************************************************************************
	Setting bits of CONFIGURATION REGISTER
    *******************************************************************************************
  */
void MCP9808_SetHysteresis(MCP9808_HandleTypeDef *mcp9808, MCP9808_Hysteresis_t hyst);

void MCP9808_SetPower(MCP9808_HandleTypeDef *mcp9808, Mcp9808_shutdown power);

void MCP9808_SetCriticalLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_crit_lock lock);

void MCP9808_SetWinLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_window_lock windowLock);

void MCP9808_SetIntClear(MCP9808_HandleTypeDef *mcp9808, MCP9808_IntClear_t intClear);

void MCP9808_SetAlertControl(MCP9808_HandleTypeDef *mcp9808, MCP9808_AlertCtrl_t alertCtrl);

void MCP9808_SetAlertPolarity(MCP9808_HandleTypeDef *mcp9808, MCP9808_AlertPolarity_t alertPolarity);

/**
    *******************************************************************************************
	Getting bits of CONFIGURATION REGISTER
    *******************************************************************************************
  */

double MCP9808_GetTemperature(MCP9808_HandleTypeDef *mcp980);

uint8_t MCP9808_GetResolution(MCP9808_HandleTypeDef *mcp9808);

void MCP9808_SetResolution(MCP9808_HandleTypeDef *mcp9808, MCP9808_Resolution_t resolution);

uint16_t MCP9808_GetDeviceId(MCP9808_HandleTypeDef *mcp9808);

uint16_t MCP9808_GetManufacturerId(MCP9808_HandleTypeDef *mcp9808);

#endif
