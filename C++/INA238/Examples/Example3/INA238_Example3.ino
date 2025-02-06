#include <INA238.h>

INA238 ina238;

static float rShuntValue    = 0.01;
static float maximumCurrent = 10.0; // The maximum current you want to measure

static float shuntUndervoltageAlert = 10e-3; 
static float shuntOverVoltageAlert  = 100e-3;
static float busUnderVoltageAlert   = 5.0;
static float busOverVoltageAlert    = 48.0;

static float shuntVoltage   = 0.0;
static float busVoltage     = 0.0;
static float current        = 0.0;
static float power          = 0.0;
static float temperature    = 0.0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  ina238.init(INA238_DEFAULT_ADDRESS, INA238_0_SEC_DELAY, INA238_40_DOT_96_MV);
  ina238.setCalibration(rShuntValue, maximumCurrent);
  ina238.initAdcConfig(INA238_CONT_TEMP_SBVOLTAGE, INA238_4120_USEC_VBUS, INA238_4120_USEC_VSH, INA238_4120_USEC_TEMP, INA238_256_COUNTS);

  ina238.setShuntUnderVoltage(shuntUndervoltageAlert);
  ina238.setShuntOverVoltage(shuntOverVoltageAlert);
  ina238.setBusUnderVoltage(busUnderVoltageAlert);
  ina238.setBusOverVoltage(busOverVoltageAlert);
}

void loop() {
  shuntVoltage = ina238.getShuntVoltage();
  busVoltage   = ina238.getBusVoltage();
  current      = ina238.getCurrent();
  power        = ina238.getPower();
  temperature  = ina238.getTemperature();

  delay(1000);
}