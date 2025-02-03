#include <BMP180.h>
#include <Wire.h>
#include "math.h"

static long X1 = 0;
static long X2 = 0;
static long X3 = 0;
static long B3 = 0;
static long B5 = 0;
static long B6 = 0;
static unsigned long B4 = 0;
static unsigned long B7 = 0;
static long p = 0;
static long altitude = 0;
static long UP = 0;

BMP180::BMP180()
{

}

uint8_t BMP180::writeRegister8(uint8_t registerAddress, uint8_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(value);
    return _wire->endTransmission();
}

uint8_t BMP180::writeRegister16(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint8_t BMP180::readRegister8(uint8_t registerAddress)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, (uint8_t)1);
    uint16_t value = _wire->read();
    return value;
}

uint16_t BMP180::readRegister16(uint8_t registerAddress)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, (uint8_t)2);
    uint16_t value = _wire->read();
    value <<= 8;    
    value |= _wire->read();
    return value;
}

uint32_t BMP180::readRegister24(uint8_t registerAddress)
{
    uint8_t registerResponse[3] = {0};

    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, (uint8_t)3);
    for (uint8_t i = 0; i < sizeof(registerResponse); i++)
    {
        registerResponse[i] = _wire->read();
    }
    uint32_t response = ((registerResponse[0] << 16) | registerResponse[1]<<8 | registerResponse[2]) >> (8 - (uint8_t)_mode);
    return response;
}

void BMP180::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire; 
    _mode = 0;
}

void BMP180::customInit(uint8_t devAddress, AccuracyMode_t accuracyMode, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire; 
    _mode = accuracyMode;
}

void BMP180::setCalibrationCoefficients()
{
    for (uint8_t reg = BMP180_CAL_COEFF_AC1; reg <= BMP180_CAL_COEFF_MD; reg++)
    {
        uint32_t value = 0;
        value = readRegister16(reg);

        switch(reg)
        {
            case BMP180_CAL_COEFF_AC1: _bmpAC1 = value;
            break;

            case BMP180_CAL_COEFF_AC2: _bmpAC2 = value;
            break;

            case BMP180_CAL_COEFF_AC3: _bmpAC3 = value;
            break;

            case BMP180_CAL_COEFF_AC4: _bmpAC4 = value;
            break;

            case BMP180_CAL_COEFF_AC5: _bmpAC5 = value;
            break;

            case BMP180_CAL_COEFF_AC6: _bmpAC6 = value;
            break;

            case BMP180_CAL_COEFF_B1: _bmpB1 = value;
            break;

            case BMP180_CAL_COEFF_B2: _bmpB2 = value;
            break;

            case BMP180_CAL_COEFF_MB: _bmpMB = value;
            break;

            case BMP180_CAL_COEFF_MC: _bmpMC = value;
            break;

            case BMP180_CAL_COEFF_MD: _bmpMD = value;
            break; 
        }
        delay(5);
    }
}

uint8_t BMP180::getId()
{
    uint8_t deviceId = readRegister8(DEVICE_ID);
    return deviceId;
}

int16_t BMP180::getCalibrationCoefficients(int16_t *calCoeff)
{
    int16_t calibrationCoeff[11] = {0};

    calibrationCoeff[0] = _bmpAC1;
    calibrationCoeff[1] = _bmpAC2;
    calibrationCoeff[2] = _bmpAC3;
    calibrationCoeff[3] = _bmpAC4;
    calibrationCoeff[4] = _bmpAC5;
    calibrationCoeff[5] = _bmpAC6;
    calibrationCoeff[6] = _bmpB1;
    calibrationCoeff[7] = _bmpB2;
    calibrationCoeff[8] = _bmpMB;
    calibrationCoeff[9] = _bmpMC;
    calibrationCoeff[10] = _bmpMD;
}

uint16_t BMP180::getUncompensatedTemperature()
{
    writeRegister8(CTRL_MEAS, 0x2E);
    delay(5);
    uint16_t response = readRegister16(OUT_MSB);

    return response;
}

long BMP180::getTemperature()
{
    uint16_t UT = getUncompensatedTemperature();

    X1 = (UT - _bmpAC6) * _bmpAC5 / pow(2, 15);
	X2 = (_bmpMC * pow(2, 11)) / (X1 + _bmpMD);
	B5 = X1 + X2;

	long temperature = ((B5 + 8) / pow(2, 4)) / 10;

	return temperature;
}

long BMP180::getUncompensatedPressure()
{
    writeRegister8(CTRL_MEAS, 0x34 + (_mode << 6) );
    delay(5);
    long response = readRegister24(OUT_MSB);

    return response;
}

long BMP180::getAltitude()
{
	UP = getUncompensatedPressure();
	B6 = B5 - 4000;
	X1 = (_bmpB2 * ((pow(B6, 2) / pow(2, 12)) )) / pow(2, 11);
	X2 = (_bmpAC2 * B6) / pow(2, 11);
	X3 = X1 + X2;
	B3 = (((_bmpAC1*4 + X3) << (uint8_t)(_mode)) + 2)/4;
	X1 =(_bmpAC3 * B6) / pow(2, 13);
	X2 = (_bmpB1 * ( pow(B6, 2) / pow(2, 12) )) / pow(2, 16);
	X3 = ((X1 + X2) + 2)/pow(2, 2);
	B4 = (_bmpAC4) * (unsigned long)(X3 + 32768) / pow(2, 15);
	B7 = ((unsigned long)UP - B3) * (50000 >> (uint8_t)(_mode));

	if (B7 < 0x80000000){
		p = ((unsigned long)B7 * 2)/B4;
	}else{
		p = (B7/B4) * 2;
	}
	X1 = (p / pow(2, 8)) * (p/ pow(2, 8));
	X1 = (X1*3038)/pow(2, 16);
	X2 = (-7357*p)/pow(2, 16);
	p = ( p + (X1 + X2 + 3791)/(pow(2, 4)) ) / 100;

	altitude = 44330 * (1 - pow(p/STD_PRESSURE, 0.190294957) );
	return altitude;  
}