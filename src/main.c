#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include the message types 
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include <rmw_microros/rmw_microros.h>


#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Include the servo library headers
#include "pico_servo.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

const uint LED_PIN = 25;
const uint SERVO_PIN = 16;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t servo_angle_publisher;
std_msgs__msg__Int32 servo_angle_msg;

rcl_subscription_t led_subscriber;
std_msgs__msg__Bool led_msg;

rcl_subscription_t servo_angle_subscriber;
std_msgs__msg__Int32 servo_angle_subscriber_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    // if (ret != RCL_RET_OK)
    // {
    //     // Handle publish error
    //     printf("Failed to publish message: %d\n", ret);
    // }
    msg.data++;
    
}

void servo_angle_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int angle = msg->data;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo_move_to(SERVO_PIN, angle);
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    servo_init();
    servo_clock_auto();

    servo_attach(SERVO_PIN);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK)
        return -1;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher"
    );

    // Timer
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback);

    // Subscriber
    rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "microROS_led_control"
    );

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &servo_angle_subscriber, &servo_angle_subscriber_msg, &servo_angle_callback, ON_NEW_DATA);
    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }

    return 0;
}
