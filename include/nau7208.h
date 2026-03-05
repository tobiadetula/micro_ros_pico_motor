#ifndef NAU7802_PICO_H
#define NAU7802_PICO_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "math.h"

// Register Map
typedef enum
{
    NAU7802_PU_CTRL = 0x00,
    NAU7802_CTRL1,
    NAU7802_CTRL2,
    NAU7802_OCAL1_B2,
    NAU7802_OCAL1_B1,
    NAU7802_OCAL1_B0,
    NAU7802_GCAL1_B3,
    NAU7802_GCAL1_B2,
    NAU7802_GCAL1_B1,
    NAU7802_GCAL1_B0,
    NAU7802_OCAL2_B2,
    NAU7802_OCAL2_B1,
    NAU7802_OCAL2_B0,
    NAU7802_GCAL2_B3,
    NAU7802_GCAL2_B2,
    NAU7802_GCAL2_B1,
    NAU7802_GCAL2_B0,
    NAU7802_I2C_CONTROL,
    NAU7802_ADCO_B2,
    NAU7802_ADCO_B1,
    NAU7802_ADCO_B0,
    NAU7802_ADC = 0x15,
    NAU7802_OTP_B1,
    NAU7802_OTP_B0,
    NAU7802_PGA = 0x1B,
    NAU7802_PGA_PWR = 0x1C,
    NAU7802_DEVICE_REV = 0x1F
} Scale_Registers;

// Gain options
typedef enum
{
    NAU7802_GAIN_128 = 0b111,
    NAU7802_GAIN_64  = 0b110,
    NAU7802_GAIN_32  = 0b101,
    NAU7802_GAIN_16  = 0b100,
    NAU7802_GAIN_8   = 0b011,
    NAU7802_GAIN_4   = 0b010,
    NAU7802_GAIN_2   = 0b001,
    NAU7802_GAIN_1   = 0b000
} NAU7802_Gain_Values;

// Sample rates
typedef enum
{
    NAU7802_SPS_320 = 0b111,
    NAU7802_SPS_80  = 0b011,
    NAU7802_SPS_40  = 0b010,
    NAU7802_SPS_20  = 0b001,
    NAU7802_SPS_10  = 0b000
} NAU7802_SPS_Values;

// Channel selection
typedef enum
{
    NAU7802_CHANNEL_1 = 0,
    NAU7802_CHANNEL_2 = 1
} NAU7802_Channels;

// Calibration status
typedef enum
{
    NAU7802_CAL_SUCCESS = 0,
    NAU7802_CAL_IN_PROGRESS = 1,
    NAU7802_CAL_FAILURE = 2
} NAU7802_Cal_Status;

// Calibration mode
typedef enum
{
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET = 2,
    NAU7802_CALMOD_GAIN = 3
} NAU7802_Cal_Mode;

class NAU7802
{
public:

    NAU7802();

    bool begin(i2c_inst_t *i2cPort = i2c0, bool reset = true);
    bool isConnected();

    bool available();
    int32_t getReading();
    int32_t getAverage(uint8_t samples);

    void calculateZeroOffset(uint8_t averageAmount = 8);
    void setZeroOffset(int32_t newZeroOffset);
    int32_t getZeroOffset();

    void calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount = 8);
    void setCalibrationFactor(float calFactor);
    float getCalibrationFactor();

    float getWeight(bool allowNegativeWeights = false, uint8_t samplesToTake = 8);

    bool setGain(uint8_t gainValue);
    bool setSampleRate(uint8_t rate);
    bool setChannel(uint8_t channelNumber);

    bool reset();
    bool powerUp();
    bool powerDown();

    uint8_t getRevisionCode();

    bool setBit(uint8_t bitNumber, uint8_t registerAddress);
    bool clearBit(uint8_t bitNumber, uint8_t registerAddress);
    bool getBit(uint8_t bitNumber, uint8_t registerAddress);

    uint8_t getRegister(uint8_t registerAddress);
    bool setRegister(uint8_t registerAddress, uint8_t value);

    int32_t get24BitRegister(uint8_t registerAddress);
    bool set24BitRegister(uint8_t registerAddress, int32_t value);

private:

    i2c_inst_t *_i2cPort;

    const uint8_t _deviceAddress = 0x2A;

    int32_t _zeroOffset = 0;
    float _calibrationFactor = 1.0;

    uint32_t _ldoRampDelay = 250;
};

/* Constructor */
void NAU7802_init(NAU7802 *dev)
{
    dev->_i2cPort = i2c0;
}

/* Begin */
bool NAU7802_begin(NAU7802 *dev, i2c_inst_t *port, bool initialize)
{
    dev->_i2cPort = port;

    if (!NAU7802_isConnected(dev))
    {
        sleep_ms(10);
        if (!NAU7802_isConnected(dev))
            return false;
    }

    bool result = true;

    if (initialize)
    {
        result &= NAU7802_reset(dev);
        result &= NAU7802_powerUp(dev);
        result &= NAU7802_setLDO(dev, NAU7802_LDO_3V3);
        result &= NAU7802_setGain(dev, NAU7802_GAIN_128);
        result &= NAU7802_setSampleRate(dev, NAU7802_SPS_80);

        uint8_t adc = NAU7802_getRegister(dev, NAU7802_ADC);
        adc |= 0x30;
        result &= NAU7802_setRegister(dev, NAU7802_ADC, adc);

        result &= NAU7802_setBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);
        result &= NAU7802_clearBit(dev, NAU7802_PGA_LDOMODE, NAU7802_PGA);

        sleep_ms(dev->_ldoRampDelay);

        NAU7802_getWeight(dev, true, 10, 1000);

        result &= NAU7802_calibrateAFE(dev, NAU7802_CALMOD_INTERNAL);
    }

    return result;
}

/* Check I2C connection */
bool NAU7802_isConnected(NAU7802 *dev)
{
    uint8_t buf;
    int ret = i2c_read_blocking(dev->_i2cPort, dev->_deviceAddress, &buf, 1, false);
    return (ret >= 0);
}

/* Data ready */
bool NAU7802_available(NAU7802 *dev)
{
    return NAU7802_getBit(dev, NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL);
}

/* Reset */
bool NAU7802_reset(NAU7802 *dev)
{
    NAU7802_setBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
    sleep_ms(1);
    return NAU7802_clearBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
}

/* Power Up */
bool NAU7802_powerUp(NAU7802 *dev)
{
    NAU7802_setBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    NAU7802_setBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

    uint8_t counter = 0;

    while (1)
    {
        if (NAU7802_getBit(dev, NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL))
            break;

        sleep_ms(1);

        if (counter++ > 100)
            return false;
    }

    return NAU7802_setBit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

/* Power Down */
bool NAU7802_powerDown(NAU7802 *dev)
{
    NAU7802_clearBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    return NAU7802_clearBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);
}

/* Set Sample Rate */
bool NAU7802_setSampleRate(NAU7802 *dev, uint8_t rate)
{
    if (rate > 0b111)
        rate = 0b111;

    uint8_t value = NAU7802_getRegister(dev, NAU7802_CTRL2);
    value &= 0b10001111;
    value |= rate << 4;

    return NAU7802_setRegister(dev, NAU7802_CTRL2, value);
}

/* Set Gain */
bool NAU7802_setGain(NAU7802 *dev, uint8_t gain)
{
    if (gain > 0b111)
        gain = 0b111;

    uint8_t value = NAU7802_getRegister(dev, NAU7802_CTRL1);
    value &= 0b11111000;
    value |= gain;

    return NAU7802_setRegister(dev, NAU7802_CTRL1, value);
}

/* Set LDO */
bool NAU7802_setLDO(NAU7802 *dev, uint8_t ldo)
{
    if (ldo > 0b111)
        ldo = 0b111;

    uint8_t value = NAU7802_getRegister(dev, NAU7802_CTRL1);
    value &= 0b11000111;
    value |= ldo << 3;

    NAU7802_setRegister(dev, NAU7802_CTRL1, value);

    return NAU7802_setBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

/* Read ADC value */
int32_t NAU7802_getReading(NAU7802 *dev)
{
    return NAU7802_get24BitRegister(dev, NAU7802_ADCO_B2);
}

/* Average readings */
int32_t NAU7802_getAverage(NAU7802 *dev, uint8_t count, uint32_t timeout_ms)
{
    int32_t total = 0;
    uint8_t samples = 0;

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (1)
    {
        if (NAU7802_available(dev))
        {
            total += NAU7802_getReading(dev);
            samples++;

            if (samples == count)
                break;
        }

        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms)
            return 0;

        sleep_ms(1);
    }

    return total / count;
}

/* Weight calculation */
float NAU7802_getWeight(NAU7802 *dev, bool allowNegative, uint8_t samples, uint32_t timeout_ms)
{
    int32_t reading = NAU7802_getAverage(dev, samples, timeout_ms);

    if (!allowNegative && reading < dev->_zeroOffset)
        reading = dev->_zeroOffset;

    float weight = (float)(reading - dev->_zeroOffset) / dev->_calibrationFactor;

    return weight;
}

/* I2C Register Read */
uint8_t NAU7802_getRegister(NAU7802 *dev, uint8_t reg)
{
    uint8_t value;

    if (i2c_write_blocking(dev->_i2cPort, dev->_deviceAddress, &reg, 1, true) < 0)
        return 0;

    if (i2c_read_blocking(dev->_i2cPort, dev->_deviceAddress, &value, 1, false) < 0)
        return 0;

    return value;
}

/* I2C Register Write */
bool NAU7802_setRegister(NAU7802 *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};

    int ret = i2c_write_blocking(dev->_i2cPort, dev->_deviceAddress, buf, 2, false);

    return ret >= 0;
}

/* Read 24-bit register */
int32_t NAU7802_get24BitRegister(NAU7802 *dev, uint8_t reg)
{
    uint8_t data[3];

    if (i2c_write_blocking(dev->_i2cPort, dev->_deviceAddress, &reg, 1, true) < 0)
        return 0;

    if (i2c_read_blocking(dev->_i2cPort, dev->_deviceAddress, data, 3, false) < 0)
        return 0;

    int32_t value =
        ((int32_t)data[0] << 16) |
        ((int32_t)data[1] << 8) |
        data[2];

    if (value & 0x00800000)
        value |= 0xFF000000;

    return value;
}

/* Bit helpers */
bool NAU7802_setBit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    uint8_t val = NAU7802_getRegister(dev, reg);
    val |= (1 << bit);
    return NAU7802_setRegister(dev, reg, val);
}

bool NAU7802_clearBit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    uint8_t val = NAU7802_getRegister(dev, reg);
    val &= ~(1 << bit);
    return NAU7802_setRegister(dev, reg, val);
}

bool NAU7802_getBit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    uint8_t val = NAU7802_getRegister(dev, reg);
    return (val & (1 << bit)) != 0;
}

#endif