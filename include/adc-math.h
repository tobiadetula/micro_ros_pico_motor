#ifndef  ADC_MATH_H
#define ADC_MATH_H


#define TEMP_AVG_WINDOW 10

static float temp_buffer[TEMP_AVG_WINDOW];
static uint8_t temp_index = 0;
static float temp_sum = 0.0f;
static bool temp_buffer_full = false;

#define TEMP_EMA_ALPHA 0.1f   // 0 < alpha <= 1

static float temp_ema = 0.0f;
static bool temp_ema_initialized = false;


static float moving_average_update(float new_sample)
{
    // Subtract oldest sample from sum
    temp_sum -= temp_buffer[temp_index];

    // Store new sample
    temp_buffer[temp_index] = new_sample;

    // Add new sample to sum
    temp_sum += new_sample;

    // Move circular index
    temp_index++;
    if (temp_index >= TEMP_AVG_WINDOW) {
        temp_index = 0;
        temp_buffer_full = true;
    }

    if (temp_buffer_full) {
        return temp_sum / TEMP_AVG_WINDOW;
    } else {
        return temp_sum / temp_index;  // Avoid divide by full window before ready
    }
}

static float ema_update(float new_sample)
{
    if (!temp_ema_initialized)
    {
        temp_ema = new_sample;   // Initialize on first sample
        temp_ema_initialized = true;
        return temp_ema;
    }

    temp_ema = TEMP_EMA_ALPHA * new_sample +
               (1.0f - TEMP_EMA_ALPHA) * temp_ema;

    return temp_ema;
}

#endif // ! ADC_MATH_H
