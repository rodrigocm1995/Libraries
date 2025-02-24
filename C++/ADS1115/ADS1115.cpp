#include <ADS1115.h>
#include <Wire.h>
#include "math.h"

// Constructor
ADS1115::ADS1115()
{

}

void ADS1115::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t ADS1115::readRegister(uint8_t registerAddress)
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

void ADS1115::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire          = wire;
    _lsbSize       = 62.5e-6; 

    uint16_t configValue = AINP_IS_AIN0_AINN_IS_AIN1 | FSR_2_DOT_048_V | ADS1115_SINGLE_SHOT | ADS1115_128_SPS | ADS1115_TRADITIONAL_COMP | ADS1115_ACTIVE_LOW | ADS1115_NONLATCHING_COMP | ADS1115_DISABLE_COMPARATOR;     _resolution = ADS1115_10_BITS_RESOLUTION;

    writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue);
}

void ADS1115::init(uint8_t devAddress, ADS115Mux muxType, ADS115Pga pga, ADS115Mode mode, ADS1115DataRate dataRate, ADS1115CompMode compMode, ADS1115CompPol pol, ADS1115CompLatch latch, ADS1115CompQueue queue, TwoWire *wire = &Wire)
{
    _deviceAddress = devAddress;
    _wire          = wire;

    uint16_t configValue = muxType | pga | mode | dataRate | compMode | pol | latch | queue;

    if (pga == FSR_6_DOT_144_V) ads1115->lsbSize = 187.5e-6;
    else if (pga == FSR_4_DOT_096_V) ads1115->lsbSize = 125e-6;
    else if (pga == FSR_2_DOT_048_V) ads1115->lsbSize = 62.5e-6;
    else if (pga == FSR_1_DOT_024_V) ads1115->lsbSize = 31.25e-6;
    else if (pga == FSR_0_DOT_512_V) ads1115->lsbSize = 15.625e-6;
    else if (pga == FSR_0_DOT_256_V) ads1115->lsbSize = 7.8125e-6;

    writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue);
}

uint16_t ADS1115::getConfig()
{
    uint16_t value = readRegister(ADS1115_CONFIGURATION_REGISTER);
    return value;
}

uint16_t ADS1115::getConversion()
{
    uint16_t value = readRegister(ADS1115_CONVERSION_REGISTER);
    return value;
}

float ADS1115::getVoltage()
{
    uint16_t value = getConversion();
    double volt = value * _lsbSize;
    return volt;
}

bool ADS1115::isDeviceReady()
{
    uint16_t config = readRegister(ADS1115_CONFIGURATION_REGISTER);
	bool isReady = CHECK_BIT(config, 15);

	if (isReady == 0)
		return 1;
	else
		return 0;
}
