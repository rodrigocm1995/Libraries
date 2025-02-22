#include <INA219.h>

INA219 ina219;

//Create these variables so that you can set the shunt resistance and maximum current you want
static float rShunt       = 0.1;
static float maxCurrent   = 3.0;

static float shuntVoltage = 0.0;
static float current      = 0.0;
static float power        = 0.0;
static float loadVoltage  = 0.0;
static float vSupply      = 0.0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);    // Start serial communication at 115200 baud

  // Initializing device
  /*
  Possible Addresses:
  INA219_I2C_ADDRESS1 = 0x40
  INA219_I2C_ADDRESS1 = 0x41
  INA219_I2C_ADDRESS1 = 0x44
  INA219_I2C_ADDRESS1 = 0x45
  */
  ina219.defaultInit(INA219_I2C_ADDRESS2);

  //Setting calibration register according Rshunt and Imax
  ina219.setCalibration(rShunt, maxCurrent); 
}

void loop() {
  // put your main code here, to run repeatedly:
    shuntVoltage = ina219.readShuntVoltage();
    loadVoltage  = ina219.readBusVoltage();
    current      = ina219.readCurrent();
    power        = ina219.readPower() * 1000.0;
    vSupply      = busVoltage + shuntVoltage/1000.0;

    Serial.print("Supply Voltage: "); Serial.print(vSupply); Serial.println(" V");
    Serial.print("Shunt Voltage:  "); Serial.print(shuntVoltage); Serial.println(" mV");
    Serial.print("Load Voltage:   "); Serial.print(loadVoltage); Serial.println(" V");
    Serial.print("Current:        "); Serial.print(current); Serial.println(" mA");
    Serial.print("Power:          "); Serial.print(power); Serial.println(" mW");
    Serial.println("");

    delay(1000);
}
