/**
 * RP2040 Firmware for ROS2 Hardware Interface
 * 
 * This firmware runs on the RP2040 microcontroller and communicates with
 * the ROS2 hardware interface over USB serial.
 * 
 * Hardware connections:
 * - Servo motor (steering) -> GPIO 0 (PWM)
 * - DC motor driver PWM -> GPIO 1, DIR -> GPIO 2
 * - IMU (I2C) -> GPIO 4 (SDA), GPIO 5 (SCL)  
 * - TF-Luna LiDAR sensors (UART) -> GPIO 8, 9, 10, 11, 12, 13
 * - Status LED -> GPIO 25 (built-in LED)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

// Pin definitions
#define SERVO_PIN 0
#define MOTOR_PWM_PIN 1  
#define MOTOR_DIR_PIN 2
#define LED_PIN 25
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// Communication protocol
#define START_MARKER '<'
#define END_MARKER '>'
#define DELIMITER ','
#define MAX_COMMAND_LENGTH 256
#define MAX_RESPONSE_LENGTH 512

// Control parameters
#define SERVO_MIN_PULSE 1000  // 1ms pulse width
#define SERVO_MAX_PULSE 2000  // 2ms pulse width  
#define SERVO_PERIOD 20000    // 20ms period
#define MOTOR_PWM_FREQ 1000   // 1kHz PWM frequency

// Global state
struct {
    // Joint states
    float joint_positions[4];      // front_left_steer, front_right_steer, rear_left_wheel, rear_right_wheel
    float joint_velocities[4];
    float joint_position_commands[4];
    float joint_velocity_commands[4];
    
    // IMU data
    struct {
        float quat_x, quat_y, quat_z, quat_w;  // quaternion
        float gyro_x, gyro_y, gyro_z;          // angular velocity (rad/s)
        float accel_x, accel_y, accel_z;       // linear acceleration (m/s^2)
    } imu;
    
    // Range sensors
    float ranges[3];  // front, left, right distances (m)
    
    // System state
    bool initialized;
    bool active;
    uint32_t last_command_time;
    
} robot_state = {0};

// Function prototypes
void init_hardware(void);
void init_pwm(void);
void init_i2c(void);
void init_uart(void);
void update_actuators(void);
void read_sensors(void);
void read_imu(void);
void read_range_sensors(void);
bool parse_command(const char* command);
void send_response(const char* response);
void send_sensor_data(void);
void process_control_command(const char* cmd);
float map_range(float value, float in_min, float in_max, float out_min, float out_max);
void simulate_sensors(void); // For testing without actual hardware

int main() {
    stdio_init_all();
    
    // Wait for USB serial connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    printf("RP2040 ROS2 Hardware Interface Starting...\n");
    
    init_hardware();
    
    printf("RP2040 Ready for commands\n");
    
    char input_buffer[MAX_COMMAND_LENGTH];
    int buffer_pos = 0;
    bool in_command = false;
    
    while (true) {
        int c = getchar_timeout_us(1000); // 1ms timeout
        
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == START_MARKER) {
                buffer_pos = 0;
                in_command = true;
            } else if (c == END_MARKER && in_command) {
                input_buffer[buffer_pos] = '\0';
                
                if (parse_command(input_buffer)) {
                    robot_state.last_command_time = time_us_32();
                }
                
                in_command = false;
                buffer_pos = 0;
            } else if (in_command && buffer_pos < MAX_COMMAND_LENGTH - 1) {
                input_buffer[buffer_pos++] = c;
            }
        }
        
        // Update control loop at ~100Hz
        static uint32_t last_update = 0;
        uint32_t now = time_us_32();
        
        if (now - last_update >= 10000) { // 10ms = 100Hz
            if (robot_state.active) {
                read_sensors();
                update_actuators();
            }
            last_update = now;
        }
        
        // Safety timeout - stop motors if no commands received
        if (robot_state.active && (now - robot_state.last_command_time) > 1000000) { // 1 second timeout
            printf("Command timeout - stopping motors\n");
            memset(robot_state.joint_velocity_commands, 0, sizeof(robot_state.joint_velocity_commands));
            robot_state.active = false;
        }
    }
    
    return 0;
}

void init_hardware(void) {
    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn on LED
    
    init_pwm();
    init_i2c();
    init_uart();
    
    // Initialize state
    robot_state.initialized = true;
    robot_state.active = false;
    robot_state.last_command_time = time_us_32();
    
    // Set initial IMU quaternion to identity
    robot_state.imu.quat_w = 1.0f;
    
    printf("Hardware initialized\n");
}

void init_pwm(void) {
    // Setup servo PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    
    // Configure for servo control (50Hz, 1-2ms pulse width)
    pwm_config servo_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&servo_config, 125.0f); // 125MHz / 125 = 1MHz
    pwm_config_set_wrap(&servo_config, SERVO_PERIOD - 1); // 20ms period
    pwm_init(servo_slice, &servo_config, true);
    
    // Setup motor PWM
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    gpio_init(MOTOR_DIR_PIN);
    gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);
    
    uint motor_slice = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    pwm_config motor_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&motor_config, 125.0f);
    pwm_config_set_wrap(&motor_config, 999); // 1kHz PWM
    pwm_init(motor_slice, &motor_config, true);
    
    printf("PWM initialized\n");
}

void init_i2c(void) {
    i2c_init(i2c0, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    printf("I2C initialized\n");
}

void init_uart(void) {
    uart_init(uart1, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    printf("UART initialized\n");
}

bool parse_command(const char* command) {
    printf("Received command: %s\n", command);
    
    if (strcmp(command, "PING") == 0) {
        send_response("<PONG>");
        return true;
    } else if (strcmp(command, "INIT") == 0) {
        robot_state.initialized = true;
        robot_state.active = false;
        printf("System initialized\n");
        send_response("<OK>");
        return true;
    } else if (strcmp(command, "STOP") == 0) {
        robot_state.active = false;
        memset(robot_state.joint_position_commands, 0, sizeof(robot_state.joint_position_commands));
        memset(robot_state.joint_velocity_commands, 0, sizeof(robot_state.joint_velocity_commands));
        printf("System stopped\n");
        send_response("<OK>");
        return true;
    } else if (strcmp(command, "READ") == 0) {
        send_sensor_data();
        return true;
    } else if (strncmp(command, "CMD", 3) == 0) {
        robot_state.active = true;
        process_control_command(command);
        send_response("<OK>");
        return true;
    }
    
    printf("Unknown command: %s\n", command);
    send_response("<ERROR>");
    return false;
}

void process_control_command(const char* cmd) {
    // Parse command format: CMD,P:val,P:val,V:val,V:val
    // Where P: indicates position command, V: indicates velocity command
    
    char* cmd_copy = malloc(strlen(cmd) + 1);
    strcpy(cmd_copy, cmd);
    
    char* token = strtok(cmd_copy, ",");
    int joint_idx = 0;
    
    // Skip "CMD" token
    token = strtok(NULL, ",");
    
    while (token != NULL && joint_idx < 4) {
        if (strncmp(token, "P:", 2) == 0) {
            float pos_cmd = atof(token + 2);
            robot_state.joint_position_commands[joint_idx] = pos_cmd;
            printf("Joint %d position command: %.3f\n", joint_idx, pos_cmd);
        } else if (strncmp(token, "V:", 2) == 0) {
            float vel_cmd = atof(token + 2);
            robot_state.joint_velocity_commands[joint_idx] = vel_cmd;
            printf("Joint %d velocity command: %.3f\n", joint_idx, vel_cmd);
        }
        
        token = strtok(NULL, ",");
        joint_idx++;
    }
    
    free(cmd_copy);
}

void update_actuators(void) {
    // Update steering servo (joint 0 - front_left_steer, joint 1 - front_right_steer)
    float steer_angle = (robot_state.joint_position_commands[0] + robot_state.joint_position_commands[1]) / 2.0f;
    
    // Map steering angle (-1 to 1) to servo pulse width (1-2ms)
    uint16_t servo_pulse = (uint16_t)map_range(steer_angle, -1.0f, 1.0f, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    uint servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_gpio_level(SERVO_PIN, servo_pulse);
    
    // Update servo positions in state
    robot_state.joint_positions[0] = steer_angle;
    robot_state.joint_positions[1] = steer_angle;
    
    // Update drive motor (joint 2 - rear_left_wheel, joint 3 - rear_right_wheel)  
    float drive_velocity = (robot_state.joint_velocity_commands[2] + robot_state.joint_velocity_commands[3]) / 2.0f;
    
    // Map velocity (-1 to 1) to PWM duty cycle
    bool forward = drive_velocity >= 0;
    uint16_t motor_pwm = (uint16_t)(fabsf(drive_velocity) * 999);
    
    gpio_put(MOTOR_DIR_PIN, forward);
    uint motor_slice = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    pwm_set_gpio_level(MOTOR_PWM_PIN, motor_pwm);
    
    // Update wheel velocities in state
    robot_state.joint_velocities[2] = drive_velocity;
    robot_state.joint_velocities[3] = drive_velocity;
}

void read_sensors(void) {
    // For now, use simulated sensors
    // In a real implementation, you would read from actual hardware
    simulate_sensors();
    
    // Uncomment these when you have real hardware:
    // read_imu();
    // read_range_sensors();
}

void simulate_sensors(void) {
    // Simulate IMU data with some realistic values
    static float sim_time = 0;
    sim_time += 0.01f; // 100Hz update rate
    
    // Simulate some motion in the IMU
    robot_state.imu.gyro_x = 0.1f * sinf(sim_time * 0.5f);
    robot_state.imu.gyro_y = 0.05f * cosf(sim_time * 0.3f);
    robot_state.imu.gyro_z = 0.02f * sinf(sim_time * 0.7f);
    
    // Simple integration for orientation (not accurate, just for simulation)
    static float roll = 0, pitch = 0, yaw = 0;
    roll += robot_state.imu.gyro_x * 0.01f;
    pitch += robot_state.imu.gyro_y * 0.01f;  
    yaw += robot_state.imu.gyro_z * 0.01f;
    
    // Convert to quaternion (simplified)
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    robot_state.imu.quat_w = cr * cp * cy + sr * sp * sy;
    robot_state.imu.quat_x = sr * cp * cy - cr * sp * sy;
    robot_state.imu.quat_y = cr * sp * cy + sr * cp * sy;
    robot_state.imu.quat_z = cr * cp * sy - sr * sp * cy;
    
    // Simulate accelerometer with gravity
    robot_state.imu.accel_x = sinf(sim_time * 0.2f) * 0.5f;
    robot_state.imu.accel_y = cosf(sim_time * 0.3f) * 0.3f;
    robot_state.imu.accel_z = 9.81f + sinf(sim_time * 0.1f) * 0.2f;
    
    // Simulate range sensors
    robot_state.ranges[0] = 2.0f + 0.5f * sinf(sim_time * 0.1f);  // front
    robot_state.ranges[1] = 1.5f + 0.3f * cosf(sim_time * 0.15f); // left
    robot_state.ranges[2] = 1.8f + 0.4f * sinf(sim_time * 0.12f); // right
}

void read_imu(void) {
    // TODO: Implement actual IMU reading (e.g., MPU6050, BNO055)
    // This would involve I2C communication to read accelerometer, gyroscope, and magnetometer
}

void read_range_sensors(void) {
    // TODO: Implement TF-Luna LiDAR reading over UART
    // This would involve UART communication to each sensor
}

void send_response(const char* response) {
    printf("%s\n", response);
}

void send_sensor_data(void) {
    char response[MAX_RESPONSE_LENGTH];
    
    // Format: <joint_pos_0,joint_vel_0,joint_pos_1,joint_vel_1,...,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,range_f,range_l,range_r>
    snprintf(response, sizeof(response),
        "<%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"  // 4 joint pos/vel pairs
        "%0.6f,%0.6f,%0.6f,%0.6f,"                          // quaternion
        "%0.6f,%0.6f,%0.6f,"                                // gyroscope  
        "%0.6f,%0.6f,%0.6f,"                                // accelerometer
        "%0.3f,%0.3f,%0.3f>",                               // range sensors
        robot_state.joint_positions[0], robot_state.joint_velocities[0],
        robot_state.joint_positions[1], robot_state.joint_velocities[1], 
        robot_state.joint_positions[2], robot_state.joint_velocities[2],
        robot_state.joint_positions[3], robot_state.joint_velocities[3],
        robot_state.imu.quat_x, robot_state.imu.quat_y, robot_state.imu.quat_z, robot_state.imu.quat_w,
        robot_state.imu.gyro_x, robot_state.imu.gyro_y, robot_state.imu.gyro_z,
        robot_state.imu.accel_x, robot_state.imu.accel_y, robot_state.imu.accel_z,
        robot_state.ranges[0], robot_state.ranges[1], robot_state.ranges[2]
    );
    
    printf("%s\n", response);
}

float map_range(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
