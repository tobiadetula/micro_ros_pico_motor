#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// Include the message types
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Include the servo library headers
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "pico_servo.h"

#include "adc-math.h"
#include "ina219.h"  // Include the INA219 library header
#include "nau7208.h" // Include the NAU7208 library header
#include "pololu-driver.h"

const uint LED_PIN = 25;
const uint PCB_LED_PIN = 7;
const uint HALL_EFFECT_PIN = 11;// Use GPIO 11 for the hall effect sensor input
// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
const float conversion_factor = 3.3f / (1 << 12);
// --- Hall Effect Global Variables ---
uint16_t previous_hall_count = 0;
// --- Actuator Positioning Variables ---
int current_motor_direction = 0; // 1 for forward, -1 for reverse, 0 for stop
int32_t absolute_position = 0;   // Signed integer to track true position
uint16_t last_pwm_count = 0;     // To calculate the change between timer ticks

// --- Targeting Variables ---
int32_t target_position = 0;
bool auto_mode = false;             // Tracks if we are driving to a target or using manual control
const int32_t POSITION_TOLERANCE = 20; // Deadband: Stop motor if within 20 ticks of target


NAU7802 scale;

// Publishers and messages
rcl_publisher_t board_temperature_publisher;
std_msgs__msg__Float32 board_temperature_msg;

rcl_publisher_t motor_current_publisher;
std_msgs__msg__Float32 motor_current_msg;

rcl_publisher_t load_cell_publisher;
std_msgs__msg__Float32 load_cell_msg;

rcl_subscription_t led_subscriber;
std_msgs__msg__Bool led_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32 motor_subscriber_msg;

// Publishers and messages for the Hall Sensor
rcl_publisher_t hall_sensor_publisher;
std_msgs__msg__Int32 hall_sensor_msg;

rcl_subscription_t target_subscriber;
std_msgs__msg__Int32 target_msg;

rcl_subscription_t tare_subscriber;
std_msgs__msg__Bool tare_msg;

rcl_subscription_t zero_position_subscriber;
std_msgs__msg__Bool zero_position_msg;


void hall_sensor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  uint slice_num = pwm_gpio_to_slice_num(HALL_EFFECT_PIN);
  uint16_t current_count = pwm_get_counter(slice_num);

  // 1. Calculate current absolute position
  uint16_t delta = current_count - last_pwm_count;
  last_pwm_count = current_count;

  if (current_motor_direction == 1) {
    absolute_position += delta;
  } else if (current_motor_direction == -1) {
    absolute_position -= delta;
  }

  // 2. Publish position
  hall_sensor_msg.data = absolute_position;
  rcl_ret_t ret = rcl_publish(&hall_sensor_publisher, &hall_sensor_msg, NULL);
  if (ret != RCL_RET_OK) {
    gpio_put(PCB_LED_PIN, 0); 
  }

  // 3. Auto-Targeting Logic
  if (auto_mode) {
    int32_t error = target_position - absolute_position;

    if (error > POSITION_TOLERANCE) {
      // We are too low, move forward
      if (current_motor_direction != 1) { // Only call motor_forward() once, don't spam it
        current_motor_direction = 1;
        motor_forward();
      }
    } 
    else if (error < -POSITION_TOLERANCE) {
      // We are too high, move backward
      if (current_motor_direction != -1) {
        current_motor_direction = -1;
        motor_reverse();
      }
    } 
    else {
      // We are inside the tolerance zone! Stop!
      if (current_motor_direction != 0) {
        current_motor_direction = 0;
        motor_stop();
        auto_mode = false; // Target reached, turn off auto mode
      }
    }
  }
}

void board_temperature_callback(rcl_timer_t *timer, int64_t last_call_time) {
  adc_select_input(4); // Select ADC channel for temperature sensor

  uint16_t raw = adc_read();                      // Read raw ADC value
  float voltage = (float)raw * conversion_factor; // Convert to voltage
  float temperature =
      27.0f -
      (voltage - 0.706f) / 0.001721f; // Convert to temperature in Celsius

  float filtered_temp = moving_average_update(temperature);
  board_temperature_msg.data = filtered_temp;
  rcl_ret_t ret =
      rcl_publish(&board_temperature_publisher, &board_temperature_msg, NULL);
  if (ret != RCL_RET_OK) {
    gpio_put(PCB_LED_PIN, 0); // turn off LED as error signal
  }
}

void motor_current_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // adc_select_input(0); // Tell the ADC to look at GPIO 26
  // float current = motor_get_current_amps();
  float current =
      fabs(ina219_read_current()); // Calculate current using shunt voltage and
                                   // shunt resistor value (0.1 Ohm)
  motor_current_msg.data = current;
  rcl_ret_t ret =
      rcl_publish(&motor_current_publisher, &motor_current_msg, NULL);
  if (ret != RCL_RET_OK) {
    gpio_put(PCB_LED_PIN, 0); // turn off LED as error signal
  }
}

// Global accumulator state
static int32_t lc_total = 0;
static uint8_t lc_samples = 0;
static bool lc_ready = false;

void load_cell_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // Take one sample per callback invocation if data is ready
  if (nau7802_available(&scale)) {
    lc_total += nau7802_get_reading(&scale);
    lc_samples++;
  }

  if (lc_samples >= SAMPLES) {
    float weight = (float)(lc_total / lc_samples - scale.zero_offset) /
                   scale.calibration_factor;
    lc_total = 0;
    lc_samples = 0;

    load_cell_msg.data = weight;
    rcl_ret_t ret = rcl_publish(&load_cell_publisher, &load_cell_msg, NULL);
    if (ret != RCL_RET_OK) {
      gpio_put(PCB_LED_PIN, 0);
    }
  }
}

void motor_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  auto_mode = false; // Turn off targeting if manual override is received
  
  if (msg->data == 1) {
    current_motor_direction = 1;
    motor_forward();
  } else if (msg->data == -1) {
    current_motor_direction = -1;
    motor_reverse();
  } else if (msg->data == 0) {
    current_motor_direction = 0;
    motor_stop();
  }
}

void led_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  gpio_put(LED_PIN, msg->data ? 1 : 0);
}

void target_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  target_position = msg->data;
  auto_mode = true; // Hand control over to the targeting system
}

void tare_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data == true) {
    // FIX: Provide a variable to store the calculated offset
    int32_t new_zero_offset = 0;
    nau7802_calculate_zero_offset(&scale, 64, &new_zero_offset); 
  }
}

void zero_position_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data == true) {
    absolute_position = 0;   // Reset the current position
    target_position = 0;     // Reset the target so it doesn't run away
    auto_mode = false;       // Turn off auto-driving
  }
}

int main() {
  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);
  /* Initialize hardware AD converter, enable onboard temperature sensor and
   *   select its channel (do this once for efficiency, but beware that this
   *   is a global operation). */
  adc_init();
  adc_gpio_init(CURRENT_SENSE_PIN); // Initialize GPIO for current sensing

  adc_set_temp_sensor_enabled(true);

  // Initialize motor driver
  motor_init();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 1); // Start with LED ON to indicate the program is running

  gpio_init(PCB_LED_PIN);
  gpio_set_dir(PCB_LED_PIN, GPIO_OUT);
  gpio_put(PCB_LED_PIN,
           1); // Start with PCB LED ON to indicate the program is running

  gpio_init(HALL_EFFECT_PIN);
  gpio_set_dir(HALL_EFFECT_PIN, GPIO_IN);
  gpio_pull_up(HALL_EFFECT_PIN); // Assuming open-drain sensor
// --- Initialize Hardware Pulse Counter ---
  // 1. Set the pin to be controlled by the PWM hardware
  gpio_set_function(HALL_EFFECT_PIN, GPIO_FUNC_PWM);
  
  // 2. Figure out which PWM slice is connected to this pin
  uint slice_num = pwm_gpio_to_slice_num(HALL_EFFECT_PIN);
  
  // 3. Get the default PWM config
  pwm_config cfg = pwm_get_default_config();
  
  // 4. MAGIC STEP: Tell it to count rising edges on the 'B' pin instead of generating a clock
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
  
  // 5. Initialize and start the hardware counter
  pwm_init(slice_num, &cfg, true);

  default_i2c_init(); // Initialize I2C for INA219 and NAU7802

  ina219_init();
  ina219_calibrate(0.1, 3.2); // Calibrate for 0.1 Ohm shunt resistor and 3.2A
                              // max expected current

  nau7802_init(&scale);
  if (!nau7802_begin(&scale, I2C_BUS, /*initialize=*/true)) {
    gpio_put(PCB_LED_PIN, 0); // turn off LED to indicate failure
                          // return -1;
  }

  // ── Normal operation: load saved calibration ──────────────────────────────
  nau7802_set_zero_offset(&scale, SAVED_ZERO_OFFSET);
  nau7802_set_calibration_factor(&scale, SAVED_CAL_FACTOR);

  // servo_init();
  // servo_clock_auto();

  // servo_attach(SERVO_PIN);

  rcl_timer_t board_temperature_timer;
  rcl_timer_t motor_current_timer;
  rcl_timer_t load_cell_timer;
  rcl_timer_t hall_sensor_timer;


  rcl_node_t node;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_executor_t executor;

  if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
    // Agent is not responding, handle error (e.g., retry, log, or indicate
    // failure)
    gpio_put(LED_PIN, 0); // turn off LED to indicate failure
                          // return -1;
  }

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_motor_node", "", &support);

  // Publisher

  rclc_publisher_init_default(
      &board_temperature_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "board_temperature");

  rclc_publisher_init_default(
      &motor_current_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_current");

  rclc_publisher_init_default(
      &load_cell_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "load_cell_weight");

  rclc_publisher_init_default(&hall_sensor_publisher, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                              "hall_sensor_counts");


  // Timer
  rclc_timer_init_default(&board_temperature_timer, &support, RCL_MS_TO_NS(500),
                          board_temperature_callback);
  rclc_timer_init_default(&motor_current_timer, &support, RCL_MS_TO_NS(100),
                          motor_current_callback);
  rclc_timer_init_default(&load_cell_timer, &support,
                          RCL_MS_TO_NS(12), // ~80 SPS
                          load_cell_callback);

  rclc_timer_init_default(&hall_sensor_timer, &support, RCL_MS_TO_NS(100),
                          hall_sensor_timer_callback);

  // Subscriber
  rclc_subscription_init_default(
      &led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");

  // ADD THIS before the executor setup
  rclc_subscription_init_default(
      &motor_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_control");

    rclc_subscription_init_default(
    &target_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "target_position");

    rclc_subscription_init_default(
      &tare_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "tare_scale");

    rclc_subscription_init_default(
      &zero_position_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "zero_position");
  // Executor
  // Initialize the executor with a capacity for 9 handles (board
  // temperature timer + motor current timer + load cell timer + hall sensor timer + 4 subscribers)
  rclc_executor_init(&executor, &support.context, 9, &allocator);
  rclc_executor_add_timer(&executor, &board_temperature_timer);
  rclc_executor_add_timer(&executor, &motor_current_timer);
  rclc_executor_add_timer(&executor, &load_cell_timer);
  rclc_executor_add_timer(&executor, &hall_sensor_timer);
  rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg,
                                 &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &motor_subscriber,
                                 &motor_subscriber_msg, &motor_callback,
                                 ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &target_subscriber,
                                 &target_msg, &target_callback,
                                 ON_NEW_DATA);
   // Add your new TARE subscriber
  rclc_executor_add_subscription(&executor, &tare_subscriber,
                                 &tare_msg, &tare_callback,
                                 ON_NEW_DATA);                         
  rclc_executor_add_subscription(&executor, &zero_position_subscriber,
                                 &zero_position_msg, &zero_position_callback,
                                 ON_NEW_DATA);
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }

  return 0;
}
