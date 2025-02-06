#include <INA238.h>

INA238 ina238;

static float rShuntValue    = 0.01;
static float maximumCurrent = 10.0; // The maximum current you want to measure

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

  ina238.init(INA238_DEFAULT_ADDRESS);
  ina238.setCalibration(rShuntValue, maximumCurrent);
}

void loop() {
  shuntVoltage = ina238.getShuntVoltage();
  busVoltage   = ina238.getBusVoltage();
  current      = ina238.getCurrent();
  power        = ina238.getPower();
  temperature  = ina238.getTemperature();

  delay(1000);
}
