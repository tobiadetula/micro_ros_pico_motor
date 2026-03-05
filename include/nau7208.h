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

#endif