/**
 * @file motor_controller.h
 * @brief PID-based motor controller for BLDC motors with ESC throttle control
 * 
 * Controls two BLDC motors via analog throttle (0-3.3V DAC output to ESCs).
 * Uses hall sensor feedback for speed measurement and PID control to maintain
 * straight-line driving and smooth turning.
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <driver/dac_oneshot.h>
#include <driver/gpio.h>

// Motor identifiers
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} motor_id_t;

// PID configuration
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float output_min;   // Min output voltage (V) - typically 0.0V
    float output_max;   // Max output voltage (V) - typically 3.3V
    float integral_max; // Anti-windup limit for integral term
} pid_config_t;

// Hall sensor configuration for one motor
typedef struct {
    gpio_num_t hall_a_pin;
    gpio_num_t hall_b_pin;
    gpio_num_t hall_c_pin;
    uint8_t pole_pairs;         // Number of magnetic pole pairs (typically 15-30 for e-bike motors)
} hall_sensor_config_t;

// Motor configuration
typedef struct {
    dac_channel_t dac_channel;          // DAC channel for throttle output
    hall_sensor_config_t hall_config;   // Hall sensor pins
    float max_rpm;                      // Maximum RPM for this motor
} motor_config_t;

// Drive input from joysticks (normalized -1.0 to +1.0)
typedef struct {
    float speed;    // Left joystick Y: -1.0 (full reverse) to +1.0 (full forward)
    float turn;     // Right joystick X: -1.0 (full left) to +1.0 (full right)
} drive_input_t;

// Motor state (read-only from external code)
typedef struct {
    float target_rpm;           // Desired RPM from joystick input
    float measured_rpm;         // Actual RPM measured from hall sensors
    float throttle_voltage;     // Current throttle voltage (0-3.3V)
    uint32_t hall_transition_count; // Total hall sensor transitions detected
    uint32_t last_transition_us;    // Timestamp of last hall transition (microseconds)
    uint8_t hall_state;         // Current hall sensor state (3 bits: CBA)
    bool enabled;               // Motor enabled/disabled
} motor_state_t;

// =============================================================================
// Public API
// =============================================================================

/**
 * @brief Initialize the motor controller system
 * 
 * Sets up DAC channels, hall sensor GPIO pins with interrupts, and PID controllers.
 * Must be called before any other motor_controller functions.
 * 
 * @param pid_cfg PID configuration (same for both motors)
 * @param motor_cfg Array of motor configurations [MOTOR_COUNT]
 * @param dac_handles Pre-initialized DAC handles for GPIO25 and GPIO26
 * @return 0 on success, -1 on error
 */
int motor_controller_init(
    const pid_config_t *pid_cfg,
    const motor_config_t motor_cfg[MOTOR_COUNT],
    dac_oneshot_handle_t dac_handles[MOTOR_COUNT]
);

/**
 * @brief Update motor control from joystick input
 * 
 * Call this from your controller data callback when joystick values change.
 * Computes target speeds using differential drive math and triggers PID update.
 * 
 * @param input Speed and turn values from joysticks (-1.0 to +1.0)
 */
void motor_controller_update_input(const drive_input_t *input);

/**
 * @brief Emergency stop - immediately set all motors to 0V throttle
 * 
 * Clears PID state and sets throttle to 0V. Motors will coast to a stop.
 * Call this when X button is pressed or on critical errors.
 */
void motor_controller_emergency_stop(void);

/**
 * @brief Enable/disable a specific motor
 * 
 * When disabled, motor throttle is set to 0V and PID is paused.
 * 
 * @param motor Motor identifier
 * @param enable true to enable, false to disable
 */
void motor_controller_enable(motor_id_t motor, bool enable);

/**
 * @brief Get current state of a motor (for debugging/telemetry)
 * 
 * @param motor Motor identifier
 * @return Pointer to motor state (read-only, valid until next PID update)
 */
const motor_state_t* motor_controller_get_state(motor_id_t motor);

/**
 * @brief Update PID tuning parameters at runtime
 * 
 * Useful for tuning PID gains while system is running.
 * 
 * @param motor Motor identifier
 * @param pid_cfg New PID configuration
 */
void motor_controller_set_pid_config(motor_id_t motor, const pid_config_t *pid_cfg);

/**
 * @brief Get current hall sensor transition count
 * 
 * Useful for odometry and debugging. Count resets on overflow (32-bit).
 * 
 * @param motor Motor identifier
 * @return Total number of hall sensor state transitions
 */
uint32_t motor_controller_get_hall_count(motor_id_t motor);

/**
 * @brief Reset hall sensor transition count to zero
 * 
 * @param motor Motor identifier
 */
void motor_controller_reset_hall_count(motor_id_t motor);

/**
 * @brief Get current measured RPM for a motor
 * 
 * @param motor Motor identifier
 * @return RPM calculated from hall sensor timing, 0.0 if motor stopped
 */
float motor_controller_get_rpm(motor_id_t motor);

/**
 * @brief Set deadzone for joystick inputs
 * 
 * Joystick values within [-deadzone, +deadzone] are treated as 0.
 * Default is 0.05 (5% deadzone).
 * 
 * @param deadzone Deadzone value (0.0 to 1.0)
 */
void motor_controller_set_deadzone(float deadzone);

/**
 * @brief Test mode: Set both motors to same target RPM and log speed tracking
 * 
 * Useful for bench testing to verify both motors respond identically.
 * Logs RPM error every 500ms. Press X button to stop.
 * 
 * @param target_rpm Target speed for both motors (e.g., 1000 RPM)
 */
void motor_controller_test_mode(float target_rpm);

/**
 * @brief Print diagnostic info for both motors
 * 
 * Logs current RPM, target RPM, throttle voltage, PID error, and hall counts.
 * Call this periodically during testing or triggered by a button.
 */
void motor_controller_print_diagnostics(void);

#endif // MOTOR_CONTROLLER_H
