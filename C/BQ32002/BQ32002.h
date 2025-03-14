#ifndef INC_BQ32002_H_
#define INC_BQ32002_H_

#define BQ32002_DEFAULT_ADDRESS                  0x48
#define BQ32002_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define BQ32002_SECONDS_REGISTER                0x00
#define BQ32002_MINUTES_REGISTER                0x01
#define BQ32002_CENT_HOURS_REGISTER             0x02
#define BQ32002_DAY_REGISTER                    0x03
#define BQ32002_DATE_REGISTER                   0x04
#define BQ32002_MONTH_REGISTER                  0x05
#define BQ32002_YEARS_REGISTER                  0x06
#define BQ32002_YEARS_REGISTER                  0x07
#define TMP117_EEPROM3_REGISTER                 0x08
#define TMP117_DEVICE_ID_REGISTER               0x0F
