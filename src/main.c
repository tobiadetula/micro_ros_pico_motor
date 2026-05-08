#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// Include the micro-ROS transport and standard types
#include <rmw_microros/rmw_microros.h>

// Include Custom ROS 2 Messages
#include <actuator_interfaces/msg/actuator_state.h>
#include <actuator_interfaces/msg/actuator_command.h>
#include <actuator_interfaces/srv/set_safeguards.h>
#include <rosidl_runtime_c/string_functions.h> // Required for string assignment

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Include the hardware library headers
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "pico_servo.h"

#include "adc-math.h"
#include "ina219.h"  
#include "nau7802.h" 
#include "pololu-driver.h"

const uint LED_PIN = 25;
const uint PCB_LED_PIN = 7;
const uint HALL_EFFECT_PIN = 11; // Use GPIO 11 for the hall effect sensor input

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
bool auto_mode = false;                // Tracks if we are driving to a target or using manual control
const int32_t POSITION_TOLERANCE = 10; // Deadband: Stop motor if within 20 ticks of target

// --- Safety limits ---
float safety_weight_limit_g    = 175.0f;  // sustained weight threshold
float safety_current_limit_ma  = 750.0f;  // sustained current threshold
float safety_didt_limit        = 50.0f;   // mA per 100ms — rate-of-change trip
uint8_t safety_sustained_ticks = 3;       // how many consecutive callbacks before stop

// --- Safety internal state ---
static float   prev_filtered_current = 0.0f;
static uint8_t weight_over_count     = 0;
static uint8_t current_over_count    = 0;
static bool    safety_tripped        = false;

static uint32_t motor_start_time_ms = 0;
const uint32_t  MOTOR_GRACE_PERIOD_MS = 300; // Ignore current spikes for 300ms upon startup

// Motor Directions 
static const int8_t MOTOR_DIR_STOP    = 0;
static const int8_t MOTOR_DIR_FORWARD = 1;
static const int8_t MOTOR_DIR_REVERSE = -1;

NAU7802 scale;

// --- Unified ROS 2 Entities ---
actuator_interfaces__msg__ActuatorState state_msg;
actuator_interfaces__msg__ActuatorCommand cmd_msg;

rcl_publisher_t state_publisher;
rcl_subscription_t command_subscriber;

// Service variables
rcl_service_t safeguard_service;
actuator_interfaces__srv__SetSafeguards_Request safeguard_req;
actuator_interfaces__srv__SetSafeguards_Response safeguard_res;

// Latest sensor readings
float latest_weight_g = 0.0f;
float latest_current_ma = 0.0f;

// Global accumulator state for Load Cell
static int32_t lc_total = 0;
static uint8_t lc_samples = 0;

// --- Function Prototypes ---
static void set_motor_state(int new_dir);
static void check_safety_and_stop(float filtered_current_ma, float weight_g);
bool check_i2c_device_alive(uint8_t address);


// --- Helper Functions ---

bool check_i2c_device_alive(uint8_t address) {
    uint8_t rxdata;
    // Attempt to read 1 byte. Timeout after 5000 microseconds to prevent hanging the whole node.
    int ret = i2c_read_timeout_us(I2C_BUS, address, &rxdata, 1, false, 5000);
    return (ret >= 0); // Returns true if the device acknowledged
}

void set_motor_state(int new_dir) {
    // If the motor is currently stopped, and we are commanding it to move, record the start time
    if (current_motor_direction == MOTOR_DIR_STOP && new_dir != MOTOR_DIR_STOP) {
        motor_start_time_ms = to_ms_since_boot(get_absolute_time());
    }
    
    current_motor_direction = new_dir;
    
    if (new_dir == MOTOR_DIR_FORWARD) {
        motor_forward();
    } else if (new_dir == MOTOR_DIR_REVERSE) {
        motor_reverse();
    } else {
        motor_stop();
    }
}

static void check_safety_and_stop(float filtered_current_ma, float weight_g) {
    if (safety_tripped) return;  // already stopped, don't re-trigger

    // Calculate if we are currently in the startup grace period
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool in_grace_period = (current_time - motor_start_time_ms) < MOTOR_GRACE_PERIOD_MS;

    // Rate-of-change (di/dt) on current
    float didt = filtered_current_ma - prev_filtered_current;
    prev_filtered_current = filtered_current_ma;

    // 🔴 ONLY check current safety if we are OUTSIDE the grace period
    if (!in_grace_period) {
        // Immediate hard stop on violent current spike
        if (didt > safety_didt_limit) {
            safety_tripped = true;
            set_motor_state(MOTOR_DIR_STOP);
            auto_mode = false;
            return;
        }

        // Sustained current threshold check
        if (filtered_current_ma > safety_current_limit_ma) {
            current_over_count++;
        } else {
            current_over_count = 0;
        }
    } else {
        // Reset current counter during grace period to prevent false accumulation
        current_over_count = 0; 
    }
    
    // 🔴 Weight check operates independently of motor startup phases
    if (weight_g > safety_weight_limit_g) {
        weight_over_count++;
    } else {
        weight_over_count = 0;
    }

    // Trip execution
    if (current_over_count >= safety_sustained_ticks || weight_over_count  >= safety_sustained_ticks) {
        safety_tripped = true;
        auto_mode = false;
        set_motor_state(MOTOR_DIR_STOP);
    }
}

// --- Callbacks ---

void command_callback(const void *msgin) {
    const actuator_interfaces__msg__ActuatorCommand *msg = 
        (const actuator_interfaces__msg__ActuatorCommand *)msgin;

    // 1. Process One-Shot Actions First
    if (msg->reset_safety) {
        safety_tripped = false;
        current_over_count = 0;
        weight_over_count = 0;
    }

    if (msg->tare_scale && state_msg.load_cell_ok) {
        // Calculates the zero offset over 64 samples (with a 1000ms timeout)
        // The library automatically saves the new offset internally!
        nau7802_calculate_zero_offset(&scale, 64, 1000); 
    }

    if (msg->zero_position) {
        absolute_position = 0;
        target_position = 0;
        auto_mode = false;
    }

    // 2. Process Motor Commands (Ignore if safety is tripped)
    if (safety_tripped) return;

    if (msg->use_auto_mode) {
        target_position = msg->target_position;
        auto_mode = true;
    }
    // If the PC sent a target position that differs from the current one, enter auto mode
    else if (msg->target_position != target_position) {
        target_position = msg->target_position;
        auto_mode = true;
    } 
    // Otherwise, process manual override modes
    else {
        auto_mode = false;
        if (msg->motor_direction == MOTOR_DIR_FORWARD) {
            set_motor_state(MOTOR_DIR_FORWARD);
        } else if (msg->motor_direction == MOTOR_DIR_REVERSE) {
            set_motor_state(MOTOR_DIR_REVERSE);
        } else if (msg->motor_direction == MOTOR_DIR_STOP) {
            set_motor_state(MOTOR_DIR_STOP);
        }
    }
}

void unified_control_loop_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time; // Unused
    
    // 1. Check Device Health on I2C Bus
    state_msg.current_sensor_ok = check_i2c_device_alive(INA219_I2C_ADDR);
    state_msg.load_cell_ok      = check_i2c_device_alive(NAU7802_I2C_ADDR);

    // 2. Read Sensors (Only if healthy, to prevent hanging or garbage data)
    if (state_msg.current_sensor_ok) {
        float raw_current = fabs(ina219_read_current()); 
        latest_current_ma = current_ema_update(raw_current);
    } else {
        latest_current_ma = 0.0f; // Default safely if sensor dies
    }

    if (state_msg.load_cell_ok && nau7802_available(&scale)) {
        lc_total += nau7802_get_reading(&scale);
        lc_samples++;
        
        // SAMPLES macro is derived from your nau7802.h
        if (lc_samples >= SAMPLES) {
            latest_weight_g = (float)(lc_total / lc_samples - scale.zero_offset) / scale.calibration_factor;
            lc_total = 0;
            lc_samples = 0;
        }
    }

    // 3. Process Hall Effect Delta
    uint slice_num = pwm_gpio_to_slice_num(HALL_EFFECT_PIN);
    uint16_t current_count = pwm_get_counter(slice_num);
    uint16_t delta = current_count - last_pwm_count;
    last_pwm_count = current_count;

    if (current_motor_direction == MOTOR_DIR_FORWARD) {
        absolute_position += delta;
    } else if (current_motor_direction == MOTOR_DIR_REVERSE) {
        absolute_position -= delta;
    }

    // 4. Run Auto-Targeting & Safety
    if (auto_mode) {
        int32_t error = target_position - absolute_position;

        if (error > POSITION_TOLERANCE) {
            if (current_motor_direction != MOTOR_DIR_FORWARD) set_motor_state(MOTOR_DIR_FORWARD);
        } 
        else if (error < -POSITION_TOLERANCE) {
            if (current_motor_direction != MOTOR_DIR_REVERSE) set_motor_state(MOTOR_DIR_REVERSE);
        } 
        else {
            if (current_motor_direction != MOTOR_DIR_STOP) {
                set_motor_state(MOTOR_DIR_STOP);
                auto_mode = false; 
            }
        }
    }
    
    if (current_motor_direction != MOTOR_DIR_STOP) {
        check_safety_and_stop(latest_current_ma, latest_weight_g);
    }

    // 5. Populate the Unified Message
    state_msg.position         = absolute_position;
    state_msg.applied_force    = latest_weight_g;
    state_msg.motor_current    = latest_current_ma;
    state_msg.motor_state      = current_motor_direction;
    state_msg.auto_mode_active = auto_mode;
    state_msg.safety_tripped   = safety_tripped;
    state_msg.safeguard_current_amps = safety_current_limit_ma / 1000.0f;
    state_msg.safeguard_force_grams  = safety_weight_limit_g;

    #ifdef ENABLE_BOARD_TEMP_PUBLISHING
    adc_select_input(4); 
    uint16_t raw = adc_read();                      
    float voltage = (float)raw * conversion_factor; 
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f; 
    state_msg.board_temp = moving_average_update(temperature);
    #else
    state_msg.board_temp = 0.0f;
    #endif

    // 6. Publish State
    rcl_ret_t ret = rcl_publish(&state_publisher, &state_msg, NULL);
    if (ret != RCL_RET_OK) {
        gpio_put(PCB_LED_PIN, 0); // Blink LED on ROS publish error
    } else {
        gpio_put(PCB_LED_PIN, 1);
    }
}

void safeguard_service_callback(const void * req, void * res) {
    // Cast the void pointers to our specific request and response types
    actuator_interfaces__srv__SetSafeguards_Request * req_in = 
        (actuator_interfaces__srv__SetSafeguards_Request *) req;
    actuator_interfaces__srv__SetSafeguards_Response * res_out = 
        (actuator_interfaces__srv__SetSafeguards_Response *) res;

    // 1. Update the safety limits (validate they are positive numbers)
    if (req_in->max_current_amps > 0.0f && req_in->max_force_grams > 0.0f) {
        // Convert Amps to mA for your internal logic
        safety_current_limit_ma = req_in->max_current_amps * 1000.0f;
        safety_weight_limit_g = req_in->max_force_grams;

        // 2. Format a Success Response
        res_out->success = true;
        rosidl_runtime_c__String__assign(&res_out->message, "Safeguards updated successfully");
    } else {
        // Format a Failure Response
        res_out->success = false;
        rosidl_runtime_c__String__assign(&res_out->message, "Error: Limits must be > 0");
    }
}

// --- Main ---

int main() {
    rmw_uros_set_custom_transport(
        true, NULL, pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read);

    // Initialize hardware AD converter
    adc_init();
    adc_gpio_init(CURRENT_SENSE_PIN); 

    #ifdef ENABLE_BOARD_TEMP_PUBLISHING
    adc_set_temp_sensor_enabled(true);
    #endif

    // Initialize motor driver
    motor_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Start with LED ON

    gpio_init(PCB_LED_PIN);
    gpio_set_dir(PCB_LED_PIN, GPIO_OUT);
    gpio_put(PCB_LED_PIN, 1); // Start with PCB LED ON 

    gpio_init(HALL_EFFECT_PIN);
    gpio_set_dir(HALL_EFFECT_PIN, GPIO_IN);
    gpio_pull_up(HALL_EFFECT_PIN);

    // --- Initialize Hardware Pulse Counter ---
    gpio_set_function(HALL_EFFECT_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(HALL_EFFECT_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_init(slice_num, &cfg, true);

    // --- Initialize I2C Devices ---
    default_i2c_init(); 

    ina219_init();
    ina219_calibrate(0.1, 3.2); 

    nau7802_init(&scale);
    if (!nau7802_begin(&scale, I2C_BUS, true)) {
        gpio_put(PCB_LED_PIN, 0); 
    }
    nau7802_set_zero_offset(&scale, SAVED_ZERO_OFFSET);
    nau7802_set_calibration_factor(&scale, SAVED_CAL_FACTOR);

    // --- ROS 2 Initialization ---
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    // Halts execution if agent is unreachable
    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
        gpio_put(LED_PIN, 0); 
        while (1) { sleep_ms(100); } // Trap execution safely
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_motor_node", "microros", &support);

    // 1 Publisher
    rclc_publisher_init_default(
        &state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_interfaces, msg, ActuatorState), 
        "actuator_telemetry"
    );

    // 1 Subscriber
    rclc_subscription_init_default(
        &command_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_interfaces, msg, ActuatorCommand), 
        "actuator_control"
    );

    // Initialize the Service
    rclc_service_init_default(
        &safeguard_service, 
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(actuator_interfaces, srv, SetSafeguards), 
        "set_safeguards"
    );

    // 1 Timer (20ms = 50Hz unified loop)
    rcl_timer_t unified_timer;
    rclc_timer_init_default(&unified_timer, &support, RCL_MS_TO_NS(20), unified_control_loop_callback);

    // Executor Initialization (Capacity of 3 handles)
    rclc_executor_init(&executor, &support.context, 3, &allocator);

    rclc_executor_add_timer(&executor, &unified_timer);
    rclc_executor_add_subscription(&executor, &command_subscriber, &cmd_msg, &command_callback, ON_NEW_DATA);

    // Add the Service Server
    rclc_executor_add_service(&executor, &safeguard_service, &safeguard_req, &safeguard_res, safeguard_service_callback);

    // Run Node
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }

    return 0;
}

//NOTE: Time for Actuator Full extension is 40s