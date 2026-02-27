#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

/* ===================== Pololu MD31C Motor Driver Connections ===================== */

/* PWM control pin (connects to MD31C PWM input)
 * - Default state: LOW
 * - A PWM signal here controls motor speed.
 * - Duty cycle directly controls average motor voltage.
 */
const uint PWM_PIN = 16;

/* Direction control pin (connects to MD31C DIR input)
 * - Default state: LOW
 * - HIGH  → Current flows OUTA → OUTB
 * - LOW   → Current flows OUTB → OUTA
 * - Determines motor rotation direction.
 */
const uint DIR_PIN = 17;

/* Sleep control pin (connects to MD31C SLP input)
 * - Default state: HIGH (driver enabled by default)
 * - Drive LOW to enter low-power sleep mode.
 * - When LOW, motor outputs are disabled.
 */
const uint SLEEP_PIN = 18;

/* Fault indicator pin (connects to MD31C FLT output)
 * - Open-drain output from driver.
 * - Goes LOW when a fault occurs (overcurrent, overtemp, etc.).
 * - Requires external pull-up resistor to logic voltage.
 */
const uint FAULT_PIN = 19;

/* Current sense pin (connects to MD31C CS output → ADC input)
 * - Outputs voltage proportional to motor current.
 * - Approx. 40 mV per amp + 50 mV offset.
 * - Valid only while actively driving (not braking).
 * - Connected here to ADC channel 0.
 */
const uint CURRENT_SENSE_PIN = 26; // ADC channel 0


// --- Function Prototypes ---
void motor_init(void);
void motor_forward(void);
void motor_reverse(void);
void motor_stop(void);
void motor_enable(bool enable);
float motor_get_current_amps(void);
bool motor_fault_active(void);



// --- ADC Constants (Based on Pololu MD31C specs) ---
#define ADC_MAX_VALUE    4095.0f // 12-bit ADC
#define ADC_REF_VOLTAGE  3.3f
#define CS_OFFSET_MV     50.0f
#define CS_MV_PER_AMP    40.0f

/* PWM control pin (connects to MD31C PWM input)
 * - Controls motor speed via duty cycle.
 * - Use gpio_put for simple on/off testing, but for variable speed use PWM functions.
 */
void motor_init()
{
    // Initialize GPIO pins
    gpio_init(PWM_PIN);
    gpio_set_dir(PWM_PIN, GPIO_OUT);
    gpio_put(PWM_PIN, 0); // Start with PWM LOW (motor stopped)

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0); // Default direction

    gpio_init(SLEEP_PIN);
    gpio_set_dir(SLEEP_PIN, GPIO_OUT);
    gpio_put(SLEEP_PIN, 1); // Start with driver enabled

    gpio_init(FAULT_PIN);
    gpio_set_dir(FAULT_PIN, GPIO_IN); // Input for fault monitoring
}

void motor_forward()
{
    gpio_put(DIR_PIN, 0); // Set direction
    gpio_put(PWM_PIN, 1); // Start motor at full speed (for testing)
}

void motor_reverse()
{
    gpio_put(DIR_PIN, 1); // Set direction
    gpio_put(PWM_PIN, 1); // Start motor at full speed (for testing)
}   

void motor_stop()
{
    gpio_put(PWM_PIN, 0); // Stop motor
}


void motor_enable(bool enable) 
{
    gpio_put(SLEEP_PIN, enable ? 1 : 0);
}

float motor_get_current_amps(void) 
{
  
    uint16_t raw = adc_read();
    float voltage = ((float)raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float mv = voltage * 1000.0f;

    float current = (mv - CS_OFFSET_MV) / CS_MV_PER_AMP;
    return (current < 0.0f) ? 0.0f : current; // Don't return negative current
}

bool motor_fault_active(void) 
{
    return (gpio_get(FAULT_PIN) == 0); // Active low
}