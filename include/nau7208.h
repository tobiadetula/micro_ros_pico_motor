#ifndef NAU7802_PICO_H
#define NAU7802_PICO_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>

// ── Calibration constants — measure these once and hard-code them ─────────────
//   1. Flash with CALIBRATE_MODE 1, place nothing on scale, note zero_offset
//   2. Flash with CALIBRATE_MODE 2, place known weight, note cal_factor
//   3. Set CALIBRATE_MODE 0 and fill in the values below
#define CALIBRATE_MODE   0        // 0 = normal, 1 = zero, 2 = gain
#define KNOWN_WEIGHT_G   500.0f   // grams on scale during gain calibration

#define SAVED_ZERO_OFFSET   0           // <-- replace after step 1
#define SAVED_CAL_FACTOR    420.0f      // <-- replace after step 2

// ── Averaging / timeout ───────────────────────────────────────────────────────
#define SAMPLES      10
#define TIMEOUT_MS   2000

// ─────────────────────────────────────────────────────────────────────────────



// ─── I2C Address ─────────────────────────────────────────────────────────────

#define NAU7802_I2C_ADDR 0x2A

// ─── Register Map ────────────────────────────────────────────────────────────

typedef enum
{
    NAU7802_PU_CTRL     = 0x00,
    NAU7802_CTRL1       = 0x01,
    NAU7802_CTRL2       = 0x02,
    NAU7802_OCAL1_B2    = 0x03,
    NAU7802_OCAL1_B1    = 0x04,
    NAU7802_OCAL1_B0    = 0x05,
    NAU7802_GCAL1_B3    = 0x06,
    NAU7802_GCAL1_B2    = 0x07,
    NAU7802_GCAL1_B1    = 0x08,
    NAU7802_GCAL1_B0    = 0x09,
    NAU7802_OCAL2_B2    = 0x0A,
    NAU7802_OCAL2_B1    = 0x0B,
    NAU7802_OCAL2_B0    = 0x0C,
    NAU7802_GCAL2_B3    = 0x0D,
    NAU7802_GCAL2_B2    = 0x0E,
    NAU7802_GCAL2_B1    = 0x0F,
    NAU7802_GCAL2_B0    = 0x10,
    NAU7802_I2C_CONTROL = 0x11,
    NAU7802_ADCO_B2     = 0x12,
    NAU7802_ADCO_B1     = 0x13,
    NAU7802_ADCO_B0     = 0x14,
    NAU7802_ADC         = 0x15,
    NAU7802_OTP_B1      = 0x16,
    NAU7802_OTP_B0      = 0x17,
    NAU7802_PGA         = 0x1B,
    NAU7802_PGA_PWR     = 0x1C,
    NAU7802_DEVICE_REV  = 0x1F
} NAU7802_Register;

// ─── PU_CTRL Bit Positions (register 0x00) ───────────────────────────────────

#define NAU7802_PU_CTRL_RR      0   // Register Reset
#define NAU7802_PU_CTRL_PUD     1   // Power Up Digital
#define NAU7802_PU_CTRL_PUA     2   // Power Up Analog
#define NAU7802_PU_CTRL_PUR     3   // Power Up Ready (read-only)
#define NAU7802_PU_CTRL_CS      4   // Cycle Start
#define NAU7802_PU_CTRL_CR      5   // Cycle Ready (data available, read-only)
#define NAU7802_PU_CTRL_OSCS    6   // System Clock Source Select
#define NAU7802_PU_CTRL_AVDDS   7   // AVDD Source Select (0 = external, 1 = internal LDO)

// ─── PGA Register Bit Positions (register 0x1B) ──────────────────────────────

#define NAU7802_PGA_LDOMODE     6   // LDO Mode (0 = accurate, 1 = less accurate)

// ─── PGA_PWR Register Bit Positions (register 0x1C) ─────────────────────────

#define NAU7802_PGA_PWR_PGA_CAP_EN  7   // PGA output capacitor enable

// ─── Gain Values (CTRL1 bits [2:0]) ─────────────────────────────────────────

typedef enum
{
    NAU7802_GAIN_1   = 0b000,
    NAU7802_GAIN_2   = 0b001,
    NAU7802_GAIN_4   = 0b010,
    NAU7802_GAIN_8   = 0b011,
    NAU7802_GAIN_16  = 0b100,
    NAU7802_GAIN_32  = 0b101,
    NAU7802_GAIN_64  = 0b110,
    NAU7802_GAIN_128 = 0b111
} NAU7802_Gain;

// ─── LDO Voltage Values (CTRL1 bits [5:3]) ───────────────────────────────────

typedef enum
{
    NAU7802_LDO_4V5 = 0b000,
    NAU7802_LDO_4V2 = 0b001,
    NAU7802_LDO_3V9 = 0b010,
    NAU7802_LDO_3V6 = 0b011,
    NAU7802_LDO_3V3 = 0b100,
    NAU7802_LDO_3V0 = 0b101,
    NAU7802_LDO_2V7 = 0b110,
    NAU7802_LDO_2V4 = 0b111
} NAU7802_LDO;

// ─── Sample Rate Values (CTRL2 bits [6:4]) ───────────────────────────────────

typedef enum
{
    NAU7802_SPS_10  = 0b000,
    NAU7802_SPS_20  = 0b001,
    NAU7802_SPS_40  = 0b010,
    NAU7802_SPS_80  = 0b011,
    NAU7802_SPS_320 = 0b111
} NAU7802_SPS;

// ─── Channel Selection (CTRL2 bit 7) ─────────────────────────────────────────

typedef enum
{
    NAU7802_CHANNEL_1 = 0,
    NAU7802_CHANNEL_2 = 1
} NAU7802_Channel;

// ─── Calibration Mode (CTRL2 bits [1:0]) ─────────────────────────────────────

typedef enum
{
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET   = 2,
    NAU7802_CALMOD_GAIN     = 3
} NAU7802_CalMode;

// ─── Calibration Status ───────────────────────────────────────────────────────

typedef enum
{
    NAU7802_CAL_SUCCESS     = 0,
    NAU7802_CAL_IN_PROGRESS = 1,
    NAU7802_CAL_FAILURE     = 2
} NAU7802_CalStatus;

// ─── Device Struct ────────────────────────────────────────────────────────────

typedef struct
{
    i2c_inst_t *i2c_port;
    uint8_t     device_address;
    int32_t     zero_offset;
    float       calibration_factor;
    uint32_t    ldo_ramp_delay_ms;
} NAU7802;

// ─── API ──────────────────────────────────────────────────────────────────────

// Initialise the struct with default values (call before begin)
void nau7802_init(NAU7802 *dev);

// Connect to device and optionally configure with sensible defaults
bool nau7802_begin(NAU7802 *dev, i2c_inst_t *port, bool initialize);

// Returns true if the device ACKs on the bus
bool nau7802_is_connected(NAU7802 *dev);

// Returns true when a new ADC conversion is ready
bool nau7802_available(NAU7802 *dev);

// Soft-reset the device
bool nau7802_reset(NAU7802 *dev);

// Power the analog and digital sections up/down
bool nau7802_power_up(NAU7802 *dev);
bool nau7802_power_down(NAU7802 *dev);

// Read the raw 24-bit two's-complement ADC value
int32_t nau7802_get_reading(NAU7802 *dev);

// Average 'count' readings (blocks up to timeout_ms milliseconds; returns 0 on timeout)
int32_t nau7802_get_average(NAU7802 *dev, uint8_t count, uint32_t timeout_ms);

// Return calibrated weight; negative weights suppressed unless allow_negative = true
float nau7802_get_weight(NAU7802 *dev, bool allow_negative, uint8_t samples, uint32_t timeout_ms);

// Store the current average reading as the zero reference
void nau7802_calculate_zero_offset(NAU7802 *dev, uint8_t average_amount, uint32_t timeout_ms);
void nau7802_set_zero_offset(NAU7802 *dev, int32_t offset);
int32_t nau7802_get_zero_offset(const NAU7802 *dev);

// Calibrate the gain factor against a known weight
void nau7802_calculate_calibration_factor(NAU7802 *dev, float weight_on_scale,
                                          uint8_t average_amount, uint32_t timeout_ms);
void nau7802_set_calibration_factor(NAU7802 *dev, float factor);
float nau7802_get_calibration_factor(const NAU7802 *dev);

// Hardware configuration
bool nau7802_set_gain(NAU7802 *dev, NAU7802_Gain gain);
bool nau7802_set_ldo(NAU7802 *dev, NAU7802_LDO ldo);
bool nau7802_set_sample_rate(NAU7802 *dev, NAU7802_SPS rate);
bool nau7802_set_channel(NAU7802 *dev, NAU7802_Channel channel);

// Run the internal AFE calibration routine
NAU7802_CalStatus nau7802_calibrate_afe(NAU7802 *dev, NAU7802_CalMode mode);

// Read the silicon revision code
uint8_t nau7802_get_revision_code(NAU7802 *dev);

// Low-level register helpers
uint8_t nau7802_get_register(NAU7802 *dev, uint8_t reg);
bool    nau7802_set_register(NAU7802 *dev, uint8_t reg, uint8_t value);
int32_t nau7802_get_24bit_register(NAU7802 *dev, uint8_t reg);
bool    nau7802_set_bit(NAU7802 *dev, uint8_t bit, uint8_t reg);
bool    nau7802_clear_bit(NAU7802 *dev, uint8_t bit, uint8_t reg);
bool    nau7802_get_bit(NAU7802 *dev, uint8_t bit, uint8_t reg);



// ─── Init ─────────────────────────────────────────────────────────────────────

void nau7802_init(NAU7802 *dev)
{
    dev->i2c_port          = i2c0;
    dev->device_address    = NAU7802_I2C_ADDR;
    dev->zero_offset       = 0;
    dev->calibration_factor = 1.0f;
    dev->ldo_ramp_delay_ms = 250;
}

// ─── Begin ────────────────────────────────────────────────────────────────────

bool nau7802_begin(NAU7802 *dev, i2c_inst_t *port, bool initialize)
{
    dev->i2c_port = port;

    // Give the device a moment if it doesn't respond immediately
    if (!nau7802_is_connected(dev))
    {
        sleep_ms(10);
        if (!nau7802_is_connected(dev))
            return false;
    }

    if (!initialize)
        return true;

    bool ok = true;

    ok &= nau7802_reset(dev);
    ok &= nau7802_power_up(dev);
    ok &= nau7802_set_ldo(dev, NAU7802_LDO_3V3);
    ok &= nau7802_set_gain(dev, NAU7802_GAIN_128);
    ok &= nau7802_set_sample_rate(dev, NAU7802_SPS_80);

    // Recommended ADC register tweak from datasheet section 9.1
    uint8_t adc = nau7802_get_register(dev, NAU7802_ADC);
    adc |= 0x30;
    ok &= nau7802_set_register(dev, NAU7802_ADC, adc);

    // Enable PGA output capacitor, use accurate LDO mode
    ok &= nau7802_set_bit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);
    ok &= nau7802_clear_bit(dev, NAU7802_PGA_LDOMODE, NAU7802_PGA);

    // Wait for the internal LDO to ramp up
    sleep_ms(dev->ldo_ramp_delay_ms);

    // Discard initial noisy readings
    nau7802_get_average(dev, 10, 1000);

    // Run internal offset/gain calibration
    NAU7802_CalStatus cal = nau7802_calibrate_afe(dev, NAU7802_CALMOD_INTERNAL);
    ok &= (cal == NAU7802_CAL_SUCCESS);

    return ok;
}

// ─── Connectivity ─────────────────────────────────────────────────────────────

bool nau7802_is_connected(NAU7802 *dev)
{
    uint8_t dummy;
    return i2c_read_blocking(dev->i2c_port, dev->device_address, &dummy, 1, false) >= 0;
}

// ─── Data Ready ───────────────────────────────────────────────────────────────

bool nau7802_available(NAU7802 *dev)
{
    return nau7802_get_bit(dev, NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL);
}

// ─── Reset ────────────────────────────────────────────────────────────────────

bool nau7802_reset(NAU7802 *dev)
{
    nau7802_set_bit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
    sleep_ms(1);
    return nau7802_clear_bit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
}

// ─── Power Up / Down ─────────────────────────────────────────────────────────

bool nau7802_power_up(NAU7802 *dev)
{
    nau7802_set_bit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    nau7802_set_bit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

    // Poll the Power-Up Ready bit (PUR) with a 100 ms timeout
    for (uint8_t i = 0; i < 100; i++)
    {
        if (nau7802_get_bit(dev, NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL))
            return nau7802_set_bit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
        sleep_ms(1);
    }
    return false;
}

bool nau7802_power_down(NAU7802 *dev)
{
    nau7802_clear_bit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    return nau7802_clear_bit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);
}

// ─── Configuration ────────────────────────────────────────────────────────────

bool nau7802_set_gain(NAU7802 *dev, NAU7802_Gain gain)
{
    uint8_t value = nau7802_get_register(dev, NAU7802_CTRL1);
    value &= 0b11111000;            // clear bits [2:0]
    value |= (uint8_t)gain & 0x07;
    return nau7802_set_register(dev, NAU7802_CTRL1, value);
}

bool nau7802_set_ldo(NAU7802 *dev, NAU7802_LDO ldo)
{
    uint8_t value = nau7802_get_register(dev, NAU7802_CTRL1);
    value &= 0b11000111;            // clear bits [5:3]
    value |= ((uint8_t)ldo & 0x07) << 3;
    nau7802_set_register(dev, NAU7802_CTRL1, value);

    // Route AVDD from the internal LDO
    return nau7802_set_bit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

bool nau7802_set_sample_rate(NAU7802 *dev, NAU7802_SPS rate)
{
    uint8_t value = nau7802_get_register(dev, NAU7802_CTRL2);
    value &= 0b10001111;            // clear bits [6:4]
    value |= ((uint8_t)rate & 0x07) << 4;
    return nau7802_set_register(dev, NAU7802_CTRL2, value);
}

bool nau7802_set_channel(NAU7802 *dev, NAU7802_Channel channel)
{
    if (channel == NAU7802_CHANNEL_1)
        return nau7802_clear_bit(dev, 7, NAU7802_CTRL2);
    else
        return nau7802_set_bit(dev, 7, NAU7802_CTRL2);
}

// ─── AFE Calibration ─────────────────────────────────────────────────────────

NAU7802_CalStatus nau7802_calibrate_afe(NAU7802 *dev, NAU7802_CalMode mode)
{
    // Write calibration mode and start bit
    uint8_t ctrl2 = nau7802_get_register(dev, NAU7802_CTRL2);
    ctrl2 &= 0b11111100;            // clear CALMOD bits [1:0]
    ctrl2 |= (uint8_t)mode & 0x03;
    ctrl2 |= (1 << 2);             // CALS bit — start calibration
    nau7802_set_register(dev, NAU7802_CTRL2, ctrl2);

    // Poll until CALS clears (calibration done) or timeout
    for (uint16_t i = 0; i < 1000; i++)
    {
        sleep_ms(1);
        uint8_t status = nau7802_get_register(dev, NAU7802_CTRL2);

        if (!(status & (1 << 2)))  // CALS cleared → done
        {
            // CAL_ERR bit (bit 3) indicates failure
            return (status & (1 << 3)) ? NAU7802_CAL_FAILURE : NAU7802_CAL_SUCCESS;
        }
    }
    return NAU7802_CAL_FAILURE;
}

// ─── Readings ─────────────────────────────────────────────────────────────────

int32_t nau7802_get_reading(NAU7802 *dev)
{
    return nau7802_get_24bit_register(dev, NAU7802_ADCO_B2);
}

int32_t nau7802_get_average(NAU7802 *dev, uint8_t count, uint32_t timeout_ms)
{
    if (count == 0)
        return 0;

    int32_t  total   = 0;
    uint8_t  samples = 0;
    uint32_t start   = to_ms_since_boot(get_absolute_time());

    while (samples < count)
    {
        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms)
            return 0;   // timed out before collecting enough samples

        if (nau7802_available(dev))
        {
            total += nau7802_get_reading(dev);
            samples++;
        }
        else
        {
            sleep_ms(1);
        }
    }

    return total / (int32_t)count;
}

// ─── Weight ───────────────────────────────────────────────────────────────────

float nau7802_get_weight(NAU7802 *dev, bool allow_negative,
                         uint8_t samples, uint32_t timeout_ms)
{
    int32_t reading = nau7802_get_average(dev, samples, timeout_ms);

    if (!allow_negative && reading < dev->zero_offset)
        reading = dev->zero_offset;

    return (float)(reading - dev->zero_offset) / dev->calibration_factor;
}

// ─── Zero / Calibration Factor ────────────────────────────────────────────────

void nau7802_calculate_zero_offset(NAU7802 *dev, uint8_t average_amount,
                                   uint32_t timeout_ms)
{
    dev->zero_offset = nau7802_get_average(dev, average_amount, timeout_ms);
}

void nau7802_set_zero_offset(NAU7802 *dev, int32_t offset)
{
    dev->zero_offset = offset;
}

int32_t nau7802_get_zero_offset(const NAU7802 *dev)
{
    return dev->zero_offset;
}

void nau7802_calculate_calibration_factor(NAU7802 *dev, float weight_on_scale,
                                          uint8_t average_amount, uint32_t timeout_ms)
{
    int32_t reading = nau7802_get_average(dev, average_amount, timeout_ms);
    dev->calibration_factor = (float)(reading - dev->zero_offset) / weight_on_scale;
}

void nau7802_set_calibration_factor(NAU7802 *dev, float factor)
{
    dev->calibration_factor = factor;
}

float nau7802_get_calibration_factor(const NAU7802 *dev)
{
    return dev->calibration_factor;
}

// ─── Revision Code ───────────────────────────────────────────────────────────

uint8_t nau7802_get_revision_code(NAU7802 *dev)
{
    return nau7802_get_register(dev, NAU7802_DEVICE_REV) & 0x0F;
}

// ─── Low-Level Register I/O ──────────────────────────────────────────────────

uint8_t nau7802_get_register(NAU7802 *dev, uint8_t reg)
{
    uint8_t value = 0;
    if (i2c_write_blocking(dev->i2c_port, dev->device_address, &reg, 1, true)  < 0) return 0;
    if (i2c_read_blocking (dev->i2c_port, dev->device_address, &value, 1, false) < 0) return 0;
    return value;
}

bool nau7802_set_register(NAU7802 *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(dev->i2c_port, dev->device_address, buf, 2, false) >= 0;
}

int32_t nau7802_get_24bit_register(NAU7802 *dev, uint8_t reg)
{
    uint8_t data[3] = {0};
    if (i2c_write_blocking(dev->i2c_port, dev->device_address, &reg, 1, true)    < 0) return 0;
    if (i2c_read_blocking (dev->i2c_port, dev->device_address, data, 3, false)   < 0) return 0;

    int32_t value = ((int32_t)data[0] << 16) |
                    ((int32_t)data[1] <<  8) |
                     (int32_t)data[2];

    // Sign-extend from 24 bits to 32 bits
    if (value & 0x00800000)
        value |= (int32_t)0xFF000000;

    return value;
}

// ─── Bit Helpers ─────────────────────────────────────────────────────────────

bool nau7802_set_bit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    uint8_t val = nau7802_get_register(dev, reg);
    val |= (uint8_t)(1u << bit);
    return nau7802_set_register(dev, reg, val);
}

bool nau7802_clear_bit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    uint8_t val = nau7802_get_register(dev, reg);
    val &= (uint8_t)~(1u << bit);
    return nau7802_set_register(dev, reg, val);
}

bool nau7802_get_bit(NAU7802 *dev, uint8_t bit, uint8_t reg)
{
    return (nau7802_get_register(dev, reg) & (1u << bit)) != 0;
}
#endif // NAU7802_PICO_H