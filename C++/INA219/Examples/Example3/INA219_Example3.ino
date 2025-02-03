#include <INA219.h>
#include <Wire.h>

INA219 ina219;

//Create these variables so that you can set the shunt resistance and maximum current you want
static float rShunt       = 0.1;
static float maxCurrent   = 2.0;

static float shuntVoltage = 0.0;
static float busVoltage   = 0.0;
static float current      = 0.0;
static float power        = 0.0;
static float loadVoltage  = 0.0;

float inaCurrent = 0.43744;
float measuredShuntCurrent = 0.4444;
bool dataReady     = 0;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);    // Start serial communication at 115200 baud
  
  //ina219.defaultInit(INA219_I2C_ADDRESS2);
  ina219.init(INA219_I2C_ADDRESS2,INA219_BUSVOLTAGERANGE_32V, INA219_PGAGAIN_320_MILI_VOLT, INA219_BUS_ADC_128SAMPLES, INA219_SHUNT_ADC_128SAMPLES, INA219_SHUNTBUS_CONTINUOUS_MODE);
  uint16_t calibration = ina219.setCalibration(rShunt, maxCurrent);
  
  ina219.correctedFullScaleCalibration(calibration, inaCurrent, measuredShuntCurrent);

  uint16_t configValue = ina219.getConfigRegister();
  uint16_t calibrationValue = ina219.getCalibrationRegister();
  Serial.print("Configuration Register Value = 0x"); Serial.println(configValue, HEX);
  Serial.print("Calibration Register Value = 0x"); Serial.println(calibrationValue, HEX);
  
}

void loop() {
  
  dataReady = ina219.dataReady();

  if (dataReady)
  {
    shuntVoltage = ina219.readShuntVoltage();
    busVoltage   = ina219.readBusVoltage();
    current      = ina219.readCurrent();
    power        = ina219.readPower() * 1000.0;
    loadVoltage  = busVoltage - (shuntVoltage/1000.0);

    Serial.print("Bus Voltage:   "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadVoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power); Serial.println(" mW");
    Serial.println("");
  }

}