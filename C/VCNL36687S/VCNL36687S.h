/**
    *******************************************************************************************

  * @file           : VCNL36687S.h
  * @brief          : VCNL36687S Library
    *******************************************************************************************

  * VCNL36687S integrates a proximity and a high power VCSEL (vertical-cavity surface-emitting laser)
  * into one small package. It incorporates photodiodes, amplifiers, and analog to digital converting
  * circuits into a single chip by CMOS process. PS programmable interrupt feature of individual high 
  * and low thresholds offers the best utilization of resource and power saving on the microcontroller.  
  * With just 20 mA pulse current, the VCNL36687S can detect the Kodak Gray Card in a distance of 20
  * cm, where other sensors need 200 mA.
  * 
  * The 12 bits proximity sensing function uses an intelligent cancellation scheme, so that cross talk
  * is eliminated effectively. To accelerate the PS response time, smart persistence prevents the
  * misjudgment of proximity sensing but also allows for a fast response time. Active force mode, one
  * one time trigger by one instruction, is a feature offering more power saving.
  *
  * PS functions are easily operated via the simple command format of I2C (SMBus compatible) interfa-
  * ce protocol. Operating voltages ranges from 1.65 V to 1.95 V. VCNL36687S is packaged in a lead
  * (Pb)-free 8 pin molding package, which offers the best market-proven reliability quality.
  * 
  * @details
  * 
  * -Package type: surface-mount
  * -Dimensions (L x W x H in mm): 3.05 x 2.00 x 1.00
  * -Integrated modules: vertical cavity surface emitting laser (VCSEL), proximity sensor (PS), and
  *  signal conditioning IC.
  * -Interrupt function
  * -Smallest light hole opening design
  * -Supply Voltage Range VDD: 1.65 V to 1.95 V
  * -Communication via I2C interface
  * -I2C bus H-level range: 1.65 V to 3.6 V
  * -Floor life: 168 h, MSL 3, according to J-STD-020
  *******************************************************************************************


Send Word -> write command to VCNL36687S
+---+---------------+----+---+--------------+---+----------------+----+----------------+---+---+
| 1 |       7       | 1  | 1 |      8       | 1 |       8        | 1  |       8        | 1 | 1 |
+---+---------------+----+---+--------------+---+----------------+----+----------------+---+---+
| S | Slave address | Wr | A | Command code | A | Data byte low  | A  | Data byte high | A | P |
+---+---------------+----+---+--------------+---+----------------+----+----------------+---+---+

Receive word -> read data from VCNL36687S

+---+---------------+----+---+--------------+---+---+---------------+----+---+---------------+---+----------------+---+---+
| 1 |       7       | 1  | 1 |      8       | 1 | 1 |       7       | 1  | 1 |       8       | 1 |       8        | 1 | 1 |
+---+---------------+----+---+--------------+---+---+---------------+----+---+---------------+---+----------------+---+---+
| S | Slave address | Wr | A | Command code | A | S | Slave address | Rd | A | Data byte low | A | Data byte high | N | P |
+---+---------------+----+---+--------------+---+---+---------------+----+---+---------------+---+----------------+---+---+

S = Start
P = Stop
A = Acknowledge
N = Not acknowledge
*/

#ifndef INC_VCNL36687S_H_
#define INC_VCNL36687S_H_

#define VCNL36687S_ADDRESS                0x60 // Single slave address
#define VCNL36687S_TRIALS                 5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define VCNL36687_PSCONFIG_1_2          0x03 /* Config 1 = LSB, Config 2 = MSB */
#define VCNL36687_PSCONFIG_3_4			0x04 /* Config 3 = LSB, Config 4 = MSB */
#define VCNL36687_CANCELLATION			0x07
#define VCNL36687_PSCONFIG_5			0x08
#define VCNL36687_PS_DATA				0xF2
#define VCNL36687_DEVICE_ID             0xF4 /* Device_ID LSB = 0x88 | Device_ID MSB = 0x05 */

// Masks
#define VCNL36687_POWER_MASK			0xFFFE
#define VCNL36687_PS_PERIOD_MASK		0xFF3F
#define VCNL36687_PS_PERS_MASK			0xFFCF
#define VCNL36687_PS_INT_MASK			0xFFF3
#define VCNL36687_PS_SMART_PERS_MASK	0xFFFD
#define VCNL36687_VCSEL_I_MASK			0xF8FF
#define VCNL36687_PS_HD_MASK			0xEFFF
#define VCNL36687_PS_IT_MASK			0x3FFF
#define VCNL36687_PS_ITB_MASK			0xF7FF

typedef enum {
	VCNL36687_PERIOD_8_MS				= 0x0000,
	VCNL36687_PERIOD_16_MS				= 0x0040,
	VCNL36687_PERIOD_32_MS				= 0x0080,
	VCNL36687_PERIOD_64_MS				= 0x00C0
}VCNL36687_Period_t;

typedef enum {
	VCNL36687_PERSISTENCE_1				= 0x0000,
	VCNL36687_PERSISTENCE_2				= 0x0010,
	VCNL36687_PERSISTENCE_3				= 0x0020,
	VCNL36687_PERSISTENCE_4				= 0x0030
}VCNL36687_Persistence_t;

typedef enum {
	VCNL36687_INT_DISABLE				= 0x0000,
	VCNL36687_INT_ENABLE				= 0x0008,
	VCNL36687_INT_TRIGGER				= 0x000C
}VCNL36687_Interrupt_t;

typedef enum {
	VCNL36687_PERSISTENCE_DISABLED		= 0x0000,
	VCNL36687_PERSISTENCE_ENABLED		= 0x0002
}VCNL36687_SmartPersistance_t;

typedef enum {
	VCNL36687_POWER_ON					= 0x0000,
	VCNL36687_POWER_OFF					= 0x0001
}VCNL36687_Power_t;

typedef enum {
	VCNL36687_PS_1T						= 0x0000,
	VCNL36687_PS_2T						= 0x4000,
	VCNL36687_PS_4T						= 0x8000,
	VCNL36687_PS_8T						= 0xC000
}VCNL36687_PS_IT_t;

typedef enum {
	VCNL36687_PS_ITB_25US				= 0x0000,
	VCNL36687_PS_ITB_50US				= 0x0800
}VCNL36687_PS_ITB_t;

typedef enum {
	VCNL36687_VCSEL_7_MA				= 0x0000,
	VCNL36687_VCSEL_11_MA				= 0x0100,
	VCNL36687_VCSEL_14_MA				= 0x0200,
	VCNL36687_VCSEL_17_MA				= 0x0300,
	VCNL36687_VCSEL_20_MA				= 0x0400
}VCNL36687_VCSEL_t;

typedef enum {
	VCNL36687_PS_12_BITS				= 0x0000,
	VCNL36687_PS_16_BITS				= 0x1000,
}VCNL36687_Resolution_t;

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            devAddress;
} VCNL36687_HandleTypeDef;  

void VCNL36687_WriteRegister(VCNL36687_HandleTypeDef *vcnl36687, uint8_t registerAddress, uint16_t value);

uint16_t VCNL36687_ReadRegister(VCNL36687_HandleTypeDef *vcnl36687, uint8_t registerAddress);

void VCNL36687_Init(VCNL36687_HandleTypeDef *vcnl36687, I2C_HandleTypeDef *i2c);

uint16_t VCNL36687_GetDeviceId(VCNL36687_HandleTypeDef *vcnl36687);

uint16_t VCNL36687_GetConfig1_2(VCNL36687_HandleTypeDef *vcnl36687);

uint16_t VCNL36687_GetConfig3_4(VCNL36687_HandleTypeDef *vcnl36687);

void VCNL36687_SetPeriod(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Period_t period);

void VCNL36687_SetPersistence(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Period_t persistence);

void VCNL36687_SetInterrupt(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Interrupt_t interrupt);

void VCNL36687_SetSmartPersistance(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_SmartPersistance_t smartPersistance);

void VCNL36687_SetPower(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Power_t power);

void VCNL36687_SetIt(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_PS_IT_t it);

void VCNL36687_SetItTime(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_PS_IT_t itTime);

void VCNL36687_SetVcselCurrent(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_VCSEL_t vcselCurrent);

void VCNL36687_SetResolution(VCNL36687_HandleTypeDef *vcnl36687, VCNL36687_Resolution_t resolution);

uint16_t VCNL36687_GetData(VCNL36687_HandleTypeDef *vcnl36687);

#endif

