/**
    *******************************************************************************************
  * @file           : AS1115.h
  * @brief          : AS1115 Library
    *******************************************************************************************

  * The AS1115 is a compact LED driver for 64 single LEDs or 8 digits of 7-segments. The 
  * devices can be programmed via an I²C compatible 2-wire interface.
  * Every segment can be individually addressed and updated sepa- rately. Only one external
  * resistor (RSET) is required to set the current. LED brightness can be controlled by 
  * analog or digital means.
  * The devices include an integrated BCD code-B/HEX decoder, multiplex scan circuitry,
  * segment and display drivers, and a 64-bit memory. Internal memory stores the shift
  * register settings, eliminating the need for continuous device reprogramming.
  * All outputs of the AS1115 can be configured for key readback. Key-switch status is
  * obtained by polling for up to 64 keys while 16 keys can be used to trigger an interrupt.
  * Additionally the AS1115 offers a diagnostic mode for easy and fast production testing.
  * The AS1115 features a low shutdown current of typically 200nA, and an operational current
  * of typically 350μA. The number of digits can be programmed, the devices can be reset by
  * software, and an external clock is also supported.and tipically consumes 3.5 µA.
  *   
  * @details
  * up to 1MHz I²C-Compatible Interface
  * Individual LED Segment Control
  * Readback for 16 Keys plus Interrupt
  * Open and Shorted LED Error Detection Global or Individual Error Detection
  * Hexadecimal- or BCD-Code for 7-Segment Displays
  * 200nA Low-Power Shutdown Current (typ.)
  * Digital and Analog Brightness Control
  * Display Blanked on Power-Up
  * Drive Common-Cathode LED displays
  * Supply Voltage Range: 2.7 V to 5.5 V
  * Software Reset
  * Optional External Clock
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_AS1115_H_
#define INC_AS1115_H_

#define AS1115_ADDRESS                  0x00
#define AS1115_TRIALS                   5

// Registers
#define AS1115_DIGIT0_REG               0x01
#define AS1115_DIGIT1_REG               0x02
#define AS1115_DIGIT2_REG               0x03
#define AS1115_DIGIT3_REG               0x04
#define AS1115_DIGIT4_REG               0x05
#define AS1115_DIGIT5_REG               0x06
#define AS1115_DIGIT6_REG               0x07
#define AS1115_DIGIT7_REG               0x08
#define AS1115_DECODE_MODE_REG          0x09
#define AS1115_INTENSITY_REG            0x0A
#define AS1115_SCAN_LIMIT_REG           0x0B
#define AS1115_SHUTDOWN_REG             0x0C
#define AS1115_SELF_ADDRESSING_REG      0x0D
#define AS1115_FEATURE_REG              0x0E
#define AS1115_DISPLAY_TEST_MODE_REG    0x0F
#define AS1115_DIG0_DIG1_INTENSITY_REG  0x10
#define AS1115_DIG2_DIG3_INTENSITY_REG  0x11
#define AS1115_DIG4_DIG5_INTENSITY_REG  0x12
#define AS1115_DIG6_DIG7_INTENSITY_REG  0x13
#define AS1115_DIAGNOSTIC_DIGIT_0       0x14
#define AS1115_DIAGNOSTIC_DIGIT_1       0x15
#define AS1115_DIAGNOSTIC_DIGIT_2       0x16
#define AS1115_DIAGNOSTIC_DIGIT_3       0x17
#define AS1115_DIAGNOSTIC_DIGIT_4       0x18
#define AS1115_DIAGNOSTIC_DIGIT_5       0x19
#define AS1115_DIAGNOSTIC_DIGIT_6       0x1A
#define AS1115_DIAGNOSTIC_DIGIT_7       0x1B

/*******************  Bits definition for DISPLAY-TEST MODE register  ******************/
#define AS1115_Display_Test_Pos         (0U)                                /* W - Optical Display Test (Test mode for external visual test) */
#define AS1115_Display_Test_Mask        (0x1U << AS1115_Display_Test_Pos)
#define AS1115_Display_Test             AS1115_Display_Test_Mask

#define AS1115_LED_Short_Pos            (1U)                                /* W - Starts a test for shorted LEDs. (Can be set together with D2) */
#define AS1115_LED_Short_Mask           (0x1U << AS1115_LED_Short_Pos)
#define AS1115_LED_Short                AS1115_LED_Short_Mask

#define AS1115_LED_Open_Pos             (2U)                                /* W - Starts a test for open LEDs. (Can be set together with D1) */
#define AS1115_LED_Open_Mask            (0x1U << AS1115_LED_Open_Pos)
#define AS1115_LED_Open                 AS1115_LED_Open_Mask
        
#define AS1115_LED_Test_Pos             (3U)                                /* R - Indicates a ongoing open/short LED test */
#define AS1115_LED_Test_Mask            (0x1U << AS1115_LED_Test_Pos)
#define AS1115_LED_Test                 AS1115_LED_Test_Mask

#define AS1115_LED_Global_Pos           (4U)                                /* R - Indicates that the last open/short LED test has detected  an error */
#define AS1115_LED_Global_Mask          (0x1U << AS1115_LED_Global_Pos)
#define AS1115_LED_Global               AS1115_LED_Global_Mask

#define AS1115_RSET_Open_Pos            (5U)                                /* R - Checks if external resistor RSET is open */
#define AS1115_RSET_Open_Mask           (0x1U << AS1115_RSET_Open_Pos)
#define AS1115_RSET_Open                AS1115_RSET_Open_Mask

#define AS1115_RSET_Short_Pos           (6U)                                /* R - Checks if external resistor RSET is shorted */
#define AS1115_RSET_Short_Mask          (0x1U << AS1115_RSET_Short_Pos)
#define AS1115_RSET_Short               AS1115_RSET_Short_Mask


typedef enum
{
  AS1115_SHUTDOWN_MODE_DF               = 0x0U, /* Reset Feature Register to Default Settings */
  AS1115_SHUTDOWN_MODE_RU               = 0x1U, /* Feature Register Unchanged */
  AS1115_NORMAL_MODE_DF                 = 0x2U, /* Reset Feature Register to Default Settings */
  AS1115_NORMAL_MODE_RU                 = 0x3U  /* Feature Register Unchanged  */
}AS1115_Mode_TypeDef;

typedef enum
{
  AS1115_DP_NORMAL_OPERATION            = 0x0U,
  AS1115_DP_RUN_DISPLAY_TEST            = 0x1U,
} AS1115_DisplayTest_TypeDef;

typedef enum
{
  AS1115_LS_NORMAL_OPERATION            = 0x0U,
  AS1115_LS_RUN_LEDSHORT_TEST           = 0x1U,
} AS1115_LedShort_TypeDef;

typedef enum
{
  AS1115_LO_NORMAL_OPERATION            = 0x0U,
  AS1115_LO_RUN_LEDOPEN_TEST            = 0x1U,
} AS1115_LedOpen_TypeDef;

typedef struct
{
    I2C_HandleTypeDef       *hi2c;
    uint8_t                 _devAddress;
} AS1115_HandleTypeDef;


/* Register Write & Read */
HAL_StatusTypeDef AS1115_WriteRegister(AS1115_HandleTypeDef *as1115, uint8_t registerAddress, uint8_t value);
uint8_t AS1115_ReadRegister(AS1115_HandleTypeDef *as1115, uint8_t registerAddress);

void AS115_SetDisplayTest(AS1115_HandleTypeDef *as1115, AS1115_DisplayTest_TypeDef displayTestMode);
void AS115_SetLedShortTest(AS1115_HandleTypeDef *as1115, AS1115_LedShort_TypeDef ledShortMode);
void AS115_SetLedOpenTest(AS1115_HandleTypeDef *as1115, AS1115_LedOpen_TypeDef ledOpenMode);

#endif
