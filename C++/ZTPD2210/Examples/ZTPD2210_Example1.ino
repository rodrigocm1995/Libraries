#include <ZTPD2210.h>

ZTPD2210 ztpd2210;

float objectTemperature = 0.0;
float ambientTemperature = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  ztpd2210.init(ZTPD2210_ADDRESS);
}

void loop() {
  // put your main code here, to run repeatedly:

  // user must read first the object temperature
  objectTemperature = ztpd2210.getObjectTemperature();
  // If user does not read the object temperature first, then he will not be able to read the ambient temperature
  ambientTemperature = ztpd2210.getAmbientTemperature();

  Serial.print("Object Temperature: ");Serial.print(objectTemperature);Serial.println(" °C");
  Serial.print("Ambient Temperature: ");Serial.print(objectTemperature);Serial.println(" °C");

  delay(1000);
}
