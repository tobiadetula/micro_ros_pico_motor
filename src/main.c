#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include the message types 
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>


#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Include the servo library headers
#include "pico_servo.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/adc.h"

#include "ina219.h"  // Include the INA219 library header
#include "adc-math.h"
#include "pololu-driver.h"

const uint LED_PIN = 25;




// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
const float conversion_factor = 3.3f / (1 << 12);

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t board_temperature_publisher;
std_msgs__msg__Float32 board_temperature_msg;

rcl_publisher_t motor_current_publisher;
std_msgs__msg__Float32 motor_current_msg;

rcl_subscription_t led_subscriber;
std_msgs__msg__Bool led_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32 motor_subscriber_msg;



void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK) {
        gpio_put(LED_PIN, 0);  // turn off LED as error signal
    }
    msg.data++;
    
}

void board_temperature_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    adc_select_input(4); // Select ADC channel for temperature sensor

    uint16_t raw = adc_read(); // Read raw ADC value
    float voltage = (float)raw * conversion_factor; // Convert to voltage
    float temperature =  27.0f - (voltage - 0.706f)/0.001721f; // Convert to temperature in Celsius

    float filtered_temp = moving_average_update(temperature);
    board_temperature_msg.data = filtered_temp;
    rcl_ret_t ret = rcl_publish(&board_temperature_publisher, &board_temperature_msg, NULL);
    if (ret != RCL_RET_OK) {
        gpio_put(LED_PIN, 0);  // turn off LED as error signal
    }
}

void motor_current_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // adc_select_input(0); // Tell the ADC to look at GPIO 26
    // float current = motor_get_current_amps();
    float current = fabs(ina219_read_current()); // Calculate current using shunt voltage and shunt resistor value (0.1 Ohm)
    motor_current_msg.data = current;
    rcl_ret_t ret = rcl_publish(&motor_current_publisher, &motor_current_msg, NULL);
    if (ret != RCL_RET_OK) {
        gpio_put(LED_PIN, 0);  // turn off LED as error signal
    }
}

void motor_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    if (msg->data == 1) {
        motor_forward();
    } else if (msg->data == 2)
     {
        motor_reverse();
    }
        else if (msg->data == 0)
        {
            motor_stop();
        }
}

void led_callback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    gpio_put(LED_PIN, msg->data ? 1 : 0);
}

int main()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    /* Initialize hardware AD converter, enable onboard temperature sensor and
     *   select its channel (do this once for efficiency, but beware that this
     *   is a global operation). */
    adc_init();
    adc_gpio_init(CURRENT_SENSE_PIN);  // Initialize GPIO for current sensing

    adc_set_temp_sensor_enabled(true);

    // Initialize motor driver
    motor_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // Start with LED ON to indicate the program is running

    ina219_i2c_init();
    ina219_init();
    ina219_calibrate(0.1, 3.2); // Calibrate for 0.1 Ohm shunt resistor and 3.2A max expected current

    // servo_init();
    // servo_clock_auto();

    // servo_attach(SERVO_PIN);

    rcl_timer_t timer;
    rcl_timer_t board_temperature_timer;
    rcl_timer_t motor_current_timer;

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK){
        // Agent is not responding, handle error (e.g., retry, log, or indicate failure)
        gpio_put(LED_PIN, 0);  // turn off LED to indicate failure
        // return -1;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_motor_node", "", &support);

    // Publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_counter"
    );

    rclc_publisher_init_default(
        &board_temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "board_temperature"
    );

    rclc_publisher_init_default(
        &motor_current_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "motor_current"
    );
    // Timer
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);
    rclc_timer_init_default(&board_temperature_timer, &support, RCL_MS_TO_NS(500), board_temperature_callback);
    rclc_timer_init_default(&motor_current_timer, &support, RCL_MS_TO_NS(100), motor_current_callback);

    // Subscriber
    rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "led_control"
    );

    // ADD THIS before the executor setup
    rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "motor_control"
    );
    // Executor
    // Initialize the executor with a capacity for 5 handles (timer + board temperature timer + motor current timer + 2 subscribers)
    rclc_executor_init(&executor, &support.context, 5, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_timer(&executor, &board_temperature_timer);
    rclc_executor_add_timer(&executor, &motor_current_timer);
    rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_subscriber_msg, &motor_callback, ON_NEW_DATA);
    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }

    return 0;
}
