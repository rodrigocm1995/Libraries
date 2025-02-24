#include <OPT3001.h>
#include <Wire.h>
#include "math.h"

// Constructor
OPT3001::OPT3001()
{

}

void OPT3001::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t OPT3001::readRegister(uint8_t registerAddress)
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

void OPT3001::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire          = wire;

    uint16_t configValue = OPT_3001_AUTOMATIC_FULL_SCALE | OPT3001_CT_800_MS | OPT3001_SHUTDOWN_MODE | LATCHED_WINDOWS_STYLE | ACTIVE_LOW_POLARITY | EXPONENT_FIELD_DISABLED | ONE_FAULT_COUNT;

    writeRegister(OPT3001_CONFIGURATION_REGISTER, configValue);
}

void OPT3001::init(uint8_t devAddress, Exponent_t exponentRange, ConversionTime_t conversionTime, ModeOfConversion_t mode, LatchField_t latchField, PolarityField_t polarity, MaskExponent_t mask, FaultCountField_t faultCount, TwoWire *wire = &Wire)
{
    _deviceAddress = devAddress;
    _wire          = wire;

    uint16_t configValue = exponentRange | conversionTime | mode | 0x0000 | latchField | polarity | mask | faultCount;

    writeRegister(OPT3001_CONFIGURATION_REGISTER, configValue);
}

void OPT3001::setLowLimit(float lowLimit)
{
    uint16_t exponent;
    uint16_t tempExponent;
    uint16_t TL; // TL[11:0] These bits are the result in straight binary coding
    uint16_t result;

    if (lowLimit <= FULL_SCALE_RANGE_40_95)
    {
      exponent = OPT3001_FSR_40DOT95;
    }
    else if (lowLimit > FULL_SCALE_RANGE_40_95 && lowLimit <= FULL_SCALE_RANGE_81_90)
    {
      exponent = OPT3001_FSR_81DOT90;
    }
    else if (lowLimit > FULL_SCALE_RANGE_81_90 && lowLimit <= FULL_SCALE_RANGE_163_80)
    {
      exponent = OPT3001_FSR_163DOT80;
    }
    else if (lowLimit > FULL_SCALE_RANGE_163_80 && lowLimit <= FULL_SCALE_RANGE_327_60)
    {
      exponent = OPT3001_FSR_327DOT60;
    }
    else if (lowLimit > FULL_SCALE_RANGE_327_60 && lowLimit <= FULL_SCALE_RANGE_655_20)
    {
      exponent = OPT3001_FSR_655DOT20;
    }
    else if (lowLimit > FULL_SCALE_RANGE_655_20 && lowLimit <= FULL_SCALE_RANGE_1310_40)
    {
      exponent = OPT3001_FSR_1310DOT40;
    }
    else if (lowLimit > FULL_SCALE_RANGE_1310_40 && lowLimit <= FULL_SCALE_RANGE_2620_80)
    {
      exponent = OPT3001_FSR_2620DOT80;
    }
    else if (lowLimit > FULL_SCALE_RANGE_2620_80 && lowLimit <= FULL_SCALE_RANGE_5241_60)
    {
      exponent = OPT3001_FSR_5241DOT60;
    }
    else if (lowLimit > FULL_SCALE_RANGE_5241_60 && lowLimit <= FULL_SCALE_RANGE_10483_20)
    {
      exponent = OPT3001_FSR_10483DOT20;
    }
    else if (lowLimit > FULL_SCALE_RANGE_10483_20 && lowLimit <= FULL_SCALE_RANGE_20966_40)
    {
      exponent = OPT3001_FSR_20966DOT40;
    }
    else if (lowLimit > FULL_SCALE_RANGE_20966_40 && lowLimit <= FULL_SCALE_RANGE_41932_80)
    {
      exponent = OPT3001_FSR_41932DOT80;
    }
    else if (lowLimit > FULL_SCALE_RANGE_41932_80 && lowLimit <= FULL_SCALE_RANGE_83865_60)
    {
      exponent = OPT3001_FSR_83865DOT60;
    }
  
    tempExponent = exponent >> 12;
    TL = lowLimit / ((0.01) * pow(2, tempExponent));
    result = exponent | TL;
  
    writeRegister(OPT3001_LOW_LIMIT_REGISTER, result);
}

float OPT3001::getLowLimit()
{
    float luxValue;

    uint16_t result = readRegister(OPT3001_LOW_LIMIT_REGISTER);
    luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);
  
    return luxValue;
}

void OPT3001:setHighLimit(float highLimit)
{
    uint16_t exponent;
    uint16_t tempExponent;
    uint16_t TH; // TL[11:0] These bits are the result in straight binary coding
    uint16_t result;
  
    if (highLimit <= FULL_SCALE_RANGE_40_95)
    {
      exponent = OPT3001_FSR_40DOT95;
    }
    else if (highLimit > FULL_SCALE_RANGE_40_95 && highLimit <= FULL_SCALE_RANGE_81_90)
    {
      exponent = OPT3001_FSR_81DOT90;
    }
    else if (highLimit > FULL_SCALE_RANGE_81_90 && highLimit <= FULL_SCALE_RANGE_163_80)
    {
      exponent = OPT3001_FSR_81DOT90;
    }
    else if (highLimit > FULL_SCALE_RANGE_163_80 && highLimit <= FULL_SCALE_RANGE_327_60)
    {
      exponent = OPT3001_FSR_327DOT60;
    }
    else if (highLimit > FULL_SCALE_RANGE_327_60 && highLimit <= FULL_SCALE_RANGE_655_20)
    {
      exponent = OPT3001_FSR_655DOT20;
    }
    else if (highLimit > FULL_SCALE_RANGE_655_20 && highLimit <= FULL_SCALE_RANGE_1310_40)
    {
      exponent = OPT3001_FSR_1310DOT40;
    }
    else if (highLimit > FULL_SCALE_RANGE_1310_40 && highLimit <= FULL_SCALE_RANGE_2620_80)
    {
      exponent = OPT3001_FSR_2620DOT80;
    }
    else if (highLimit > FULL_SCALE_RANGE_2620_80 && highLimit <= FULL_SCALE_RANGE_5241_60)
    {
      exponent = OPT3001_FSR_5241DOT60;
    }
    else if (highLimit > FULL_SCALE_RANGE_5241_60 && highLimit <= FULL_SCALE_RANGE_10483_20)
    {
      exponent = OPT3001_FSR_10483DOT20;
    }
    else if (highLimit > FULL_SCALE_RANGE_10483_20 && highLimit <= FULL_SCALE_RANGE_20966_40)
    {
      exponent = OPT3001_FSR_20966DOT40;
    }
    else if (highLimit > FULL_SCALE_RANGE_20966_40 && highLimit <= FULL_SCALE_RANGE_41932_80)
    {
      exponent = OPT3001_FSR_41932DOT80;
    }
    else if (highLimit > FULL_SCALE_RANGE_41932_80 && highLimit <= FULL_SCALE_RANGE_83865_60)
    {
      exponent = OPT3001_FSR_83865DOT60;
    }
  
    tempExponent = exponent >> 12;
    TH = highLimit / ((0.01) * pow(2, tempExponent));
    result = exponent | TH;
  
    writeRegister(OPT3001_HIGH_LIMIT_REGISTER, result);
}

float OPT3001::getHighLimit()
{
    float luxValue;

    uint16_t result = readRegister(OPT3001_HIGH_LIMIT_REGISTER);
    luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);
  
    return luxValue;
}

uint16_t OPT3001::getManufacturerId()
{
    uint16_t result = readRegister(OPT3001_MANUFACTURER_ID_REGISTER);
    return result;
}

uint16_t OPT3001::getConfig()
{
    uint16_t result;
    result = readRegister(OPT3001_CONFIGURATION_REGISTER);
  
    return result;
}

float OPT3001::getLux()
{
    uint16_t result;
    double luxValue;
  
    result = readRegister(OPT3001_RESULT_REGISTER);
    luxValue = (0.01) * pow(2, (result >> 12)) * (result & 0x0FFF);
  
    return luxValue;
}

bool OPT3001::isLowlimit()
{
    uint16_t result = readRegister(OPT3001_CONFIGURATION_REGISTER);
    _Bool isLowLimitFlagTrue = CHECK_BIT(result, FLAG_LOW_FIELD_BIT);
  
    return isLowLimitFlagTrue;
}

bool OPT3001::isHighlimit()
{
    uint16_t result = readRegister(OPT3001_CONFIGURATION_REGISTER);
    _Bool isHighLimitFlagTrue = CHECK_BIT(result, FLAG_HIGH_FIELD_BIT);
  
    return isHighLimitFlagTrue;
}

