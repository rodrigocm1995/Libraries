#include <Wire.h>
#include <TMP117.h>

static float highTemperatureLimit = 35.0;
static float lowTemperatureLimit  = 20.0;
static float currentTemperature   = 0.0;

static float highLimit;
static float lowLimit;

bool arr[3] = {0};

TMP117 tmp117;

void setup() {
  // put your setup code here, to run once:
  //Wire.begin();
  delay(100);
  Serial.begin(115200);

  tmp117.init(TMP117_ADDRESS1);
  tmp117.setHighLimit(highTemperatureLimit);
  tmp117.setLowLimit(lowTemperatureLimit);

  /*
  highLimit = tmp117.getLowLimit();
  lowLimit = tmp117.getHighLimit();
  Serial.print("High Limit: "); Serial.println(highLimit, DEC);
  Serial.print("Low Limit: "); Serial.println(lowLimit, DEC);
  */
}

void loop() {
  // put your main code here, to run repeatedly:
  tmp117.alertAndDataReady(arr);

  if (arr[2])
  {
    currentTemperature = tmp117.getTemperature();
    Serial.print("Temperature: "); Serial.println(currentTemperature);
  }
}
