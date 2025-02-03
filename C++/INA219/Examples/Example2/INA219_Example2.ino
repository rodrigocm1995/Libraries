#include <INA219.h>
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);    // Start serial communication at 115200 baud

  //If you want a custom configuration for the INA219 then call init() method
  ina219.init(INA219_I2C_ADDRESS2,INA219_BUSVOLTAGERANGE_32V, INA219_PGAGAIN_320_MILI_VOLT, INA219_BUS_ADC_128SAMPLES, INA219_SHUNT_ADC_128SAMPLES, INA219_SHUNTBUS_CONTINUOUS_MODE);
  //Setting calibration register according Rshunt and Imax
  ina219.setCalibration(rShunt, maxCurrent);

}

void loop() {
  // put your main code here, to run repeatedly:
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

  delay(1000);
}
