/**
 * @file motor_controller.c
 * @brief PID-based motor controller implementation for BLDC motors
 * 
 * Controls two BLDC motors via analog throttle (0-3.3V DAC) with hall sensor feedback.
 */

#include "motor_controller.h"
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <driver/gpio.h>

// Logging tag
static const char* TAG = "motor_ctrl";

// Pin assignments (ESP32-WROOM compatible)
// Motor Left Hall Sensors
#define MOTOR_LEFT_HALL_A   GPIO_NUM_32
#define MOTOR_LEFT_HALL_B   GPIO_NUM_33
#define MOTOR_LEFT_HALL_C   GPIO_NUM_34

// Motor Right Hall Sensors (using GPIO14, 27 since WROOM only goes to GPIO35)
#define MOTOR_RIGHT_HALL_A  GPIO_NUM_35
#define MOTOR_RIGHT_HALL_B  GPIO_NUM_14
#define MOTOR_RIGHT_HALL_C  GPIO_NUM_27

// Control parameters
#define PID_LOOP_PERIOD_MS      20      // 50Hz update rate
#define HALL_TIMEOUT_MS         1000    // Consider stopped if no transitions for 1s
#define MIN_RPM_THRESHOLD       10.0f   // Below this is considered stopped
#define DEFAULT_DEADZONE        0.05f   // 5% joystick deadzone
#define DEFAULT_POLE_PAIRS      1       // Set to 1 initially to measure actual pole pairs

// PID controller state
typedef struct {
    // PID gains
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;
    
    // PID state
    float integral;
    float prev_error;
    uint64_t last_update_us;
} pid_state_t;

// Internal motor state (extends public motor_state_t)
typedef struct {
    motor_state_t public_state;     // Public read-only state
    pid_state_t pid;                // PID controller state
    dac_oneshot_handle_t dac_handle; // DAC handle for throttle output
    gpio_num_t hall_pins[3];        // Hall sensor GPIO pins [A, B, C]
    uint8_t pole_pairs;             // Number of pole pairs
    volatile uint32_t hall_count;   // Hall transition counter (updated in ISR)
    volatile uint64_t last_hall_time_us; // Timestamp of last hall transition (ISR)
    uint64_t prev_hall_count;       // For RPM calculation
    uint64_t prev_rpm_calc_time_us; // Last time RPM was calculated
} motor_internal_t;

// Global state
static motor_internal_t g_motors[MOTOR_COUNT];
static TaskHandle_t g_pid_task_handle = NULL;
static bool g_initialized = false;
static float g_deadzone = DEFAULT_DEADZONE;
static drive_input_t g_current_input = {0};

// Forward declarations
static void IRAM_ATTR hall_sensor_isr(void* arg);
static void pid_control_task(void* pvParameters);
static float calculate_rpm(motor_id_t motor);
static float pid_compute(motor_id_t motor, float target_rpm);
static void set_throttle_voltage(motor_id_t motor, float voltage);
static float apply_deadzone(float value, float deadzone);

// =============================================================================
// Public API Implementation
// =============================================================================

int motor_controller_init(
    const pid_config_t *pid_cfg,
    const motor_config_t motor_cfg[MOTOR_COUNT],
    dac_oneshot_handle_t dac_handles[MOTOR_COUNT]
) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return 0;
    }

    ESP_LOGI(TAG, "Initializing motor controller...");

    // Initialize motor structures
    memset(g_motors, 0, sizeof(g_motors));

    // Configure each motor
    const gpio_num_t hall_pins[MOTOR_COUNT][3] = {
        {MOTOR_LEFT_HALL_A, MOTOR_LEFT_HALL_B, MOTOR_LEFT_HALL_C},
        {MOTOR_RIGHT_HALL_A, MOTOR_RIGHT_HALL_B, MOTOR_RIGHT_HALL_C}
    };

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_internal_t* motor = &g_motors[i];
        
        // Copy configuration
        motor->dac_handle = dac_handles[i];
        motor->pole_pairs = motor_cfg[i].hall_config.pole_pairs > 0 ? 
                           motor_cfg[i].hall_config.pole_pairs : DEFAULT_POLE_PAIRS;
        motor->public_state.enabled = true;
        
        memcpy(motor->hall_pins, hall_pins[i], sizeof(motor->hall_pins));
        
        // Initialize PID
        motor->pid.kp = pid_cfg->kp;
        motor->pid.ki = pid_cfg->ki;
        motor->pid.kd = pid_cfg->kd;
        motor->pid.output_min = pid_cfg->output_min;
        motor->pid.output_max = pid_cfg->output_max;
        motor->pid.integral_max = pid_cfg->integral_max;
        
        // Configure hall sensor GPIO pins
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,  // Trigger on both edges
        };
        
        for (int j = 0; j < 3; j++) {
            io_conf.pin_bit_mask = (1ULL << motor->hall_pins[j]);
            gpio_config(&io_conf);
            
            // Install ISR handler
            gpio_isr_handler_add(motor->hall_pins[j], hall_sensor_isr, (void*)(intptr_t)i);
        }
        
        // Set initial throttle to 0V
        set_throttle_voltage(i, 0.0f);
        
        ESP_LOGI(TAG, "Motor %d: Hall pins [%d,%d,%d], pole_pairs=%d",
                 i, motor->hall_pins[0], motor->hall_pins[1], motor->hall_pins[2],
                 motor->pole_pairs);
    }

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Create PID control task
    xTaskCreatePinnedToCore(
        pid_control_task,
        "pid_ctrl",
        4096,
        NULL,
        10,  // High priority for control loop
        &g_pid_task_handle,
        1    // Run on core 1 (separate from Bluetooth on core 0)
    );

    g_initialized = true;
    ESP_LOGI(TAG, "Motor controller initialized successfully");
    return 0;
}

void motor_controller_update_input(const drive_input_t *input) {
    if (!g_initialized) return;
    
    // Apply deadzone
    g_current_input.speed = apply_deadzone(input->speed, g_deadzone);
    g_current_input.turn = apply_deadzone(input->turn, g_deadzone);
    
    // Differential drive calculation will happen in PID task
}

void motor_controller_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP!");
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Set throttle to 0V immediately
        set_throttle_voltage(i, 0.0f);
        
        // Reset PID state
        g_motors[i].pid.integral = 0.0f;
        g_motors[i].pid.prev_error = 0.0f;
        g_motors[i].public_state.target_rpm = 0.0f;
    }
    
    // Clear input
    g_current_input.speed = 0.0f;
    g_current_input.turn = 0.0f;
}

void motor_controller_enable(motor_id_t motor, bool enable) {
    if (motor >= MOTOR_COUNT) return;
    
    g_motors[motor].public_state.enabled = enable;
    
    if (!enable) {
        set_throttle_voltage(motor, 0.0f);
        g_motors[motor].pid.integral = 0.0f;
        g_motors[motor].pid.prev_error = 0.0f;
    }
    
    ESP_LOGI(TAG, "Motor %d %s", motor, enable ? "enabled" : "disabled");
}

const motor_state_t* motor_controller_get_state(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return NULL;
    return &g_motors[motor].public_state;
}

void motor_controller_set_pid_config(motor_id_t motor, const pid_config_t *pid_cfg) {
    if (motor >= MOTOR_COUNT) return;
    
    motor_internal_t* m = &g_motors[motor];
    m->pid.kp = pid_cfg->kp;
    m->pid.ki = pid_cfg->ki;
    m->pid.kd = pid_cfg->kd;
    m->pid.output_min = pid_cfg->output_min;
    m->pid.output_max = pid_cfg->output_max;
    m->pid.integral_max = pid_cfg->integral_max;
    
    ESP_LOGI(TAG, "Motor %d: Updated PID (kp=%.3f, ki=%.3f, kd=%.3f)",
             motor, pid_cfg->kp, pid_cfg->ki, pid_cfg->kd);
}

uint32_t motor_controller_get_hall_count(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return g_motors[motor].hall_count;
}

void motor_controller_reset_hall_count(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return;
    g_motors[motor].hall_count = 0;
    g_motors[motor].prev_hall_count = 0;
}

float motor_controller_get_rpm(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return 0.0f;
    return g_motors[motor].public_state.measured_rpm;
}

void motor_controller_set_deadzone(float deadzone) {
    g_deadzone = fmaxf(0.0f, fminf(1.0f, deadzone));
    ESP_LOGI(TAG, "Deadzone set to %.2f", g_deadzone);
}


/* 
 * TEST MODE
 */
void motor_controller_test_mode(float target_rpm) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Motor controller not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== TEST MODE: Setting both motors to %.0f RPM ===", target_rpm);
    ESP_LOGI(TAG, "Watch for speed tracking. Press X button to stop.");
    
    // Set both motors to same target
    drive_input_t test_input = {
        .speed = target_rpm / 4500.0f,  // Normalize to 0-1 range
        .turn = 0.0f                     // No turning
    };
    
    motor_controller_update_input(&test_input);
    
    // Log speed comparison every 500ms for 10 seconds or until stopped
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        
        float left_rpm = motor_controller_get_rpm(MOTOR_LEFT);
        float right_rpm = motor_controller_get_rpm(MOTOR_RIGHT);
        float rpm_diff = fabsf(left_rpm - right_rpm);
        float avg_rpm = (left_rpm + right_rpm) / 2.0f;
        float error_percent = (avg_rpm > 0) ? (rpm_diff / avg_rpm * 100.0f) : 0.0f;
        
        ESP_LOGI(TAG, "[%.1fs] L: %.0f RPM, R: %.0f RPM | Diff: %.0f RPM (%.1f%%) | L_V: %.2fV, R_V: %.2fV",
                 (i + 1) * 0.5f,
                 left_rpm, right_rpm, rpm_diff, error_percent,
                 g_motors[MOTOR_LEFT].public_state.throttle_voltage,
                 g_motors[MOTOR_RIGHT].public_state.throttle_voltage);
        
        // Good tracking if within 5% error
        if (error_percent < 5.0f && avg_rpm > 100.0f) {
            ESP_LOGI(TAG, "✓ Motors tracking well (< 5%% error)");
        } else if (error_percent > 10.0f) {
            ESP_LOGW(TAG, "⚠ Large speed difference - check motors/wiring");
        }
    }
    
    ESP_LOGI(TAG, "=== TEST MODE COMPLETE ===");
}

void motor_controller_print_diagnostics(void) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Motor controller not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== MOTOR DIAGNOSTICS ===");
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const char* name = (i == MOTOR_LEFT) ? "LEFT" : "RIGHT";
        motor_internal_t* m = &g_motors[i];
        float error = m->public_state.target_rpm - m->public_state.measured_rpm;
        
        ESP_LOGI(TAG, "[%s Motor]", name);
        ESP_LOGI(TAG, "  Status: %s", m->public_state.enabled ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "  Target RPM: %.0f", m->public_state.target_rpm);
        ESP_LOGI(TAG, "  Measured RPM: %.0f", m->public_state.measured_rpm);
        ESP_LOGI(TAG, "  Error: %.0f RPM (%.1f%%)", 
                 error, 
                 (m->public_state.target_rpm > 0) ? (error / m->public_state.target_rpm * 100.0f) : 0.0f);
        ESP_LOGI(TAG, "  Throttle: %.2fV", m->public_state.throttle_voltage);
        ESP_LOGI(TAG, "  Hall count: %u", m->hall_count);
        ESP_LOGI(TAG, "  Hall state: 0x%02X", m->public_state.hall_state);
        ESP_LOGI(TAG, "  Pole pairs: %d", m->pole_pairs);
        ESP_LOGI(TAG, "  PID: kp=%.3f, ki=%.3f, kd=%.3f, I=%.2f",
                 m->pid.kp, m->pid.ki, m->pid.kd, m->pid.integral);
    }
    
    ESP_LOGI(TAG, "=========================");
}

// =============================================================================
// Internal Functions
// =============================================================================

static void IRAM_ATTR hall_sensor_isr(void* arg) {
    motor_id_t motor = (motor_id_t)(intptr_t)arg;
    motor_internal_t* m = &g_motors[motor];
    
    // Increment transition counter
    m->hall_count++;
    
    // Record timestamp
    m->last_hall_time_us = esp_timer_get_time();
    
    // Read current hall state (for debugging/validation)
    uint8_t state = 0;
    state |= gpio_get_level(m->hall_pins[0]) ? 0x01 : 0x00;
    state |= gpio_get_level(m->hall_pins[1]) ? 0x02 : 0x00;
    state |= gpio_get_level(m->hall_pins[2]) ? 0x04 : 0x00;
    m->public_state.hall_state = state;
}

static float calculate_rpm(motor_id_t motor) {
    motor_internal_t* m = &g_motors[motor];
    uint64_t now_us = esp_timer_get_time();
    
    // Check for timeout (motor stopped)
    if ((now_us - m->last_hall_time_us) > (HALL_TIMEOUT_MS * 1000)) {
        return 0.0f;
    }
    
    // Calculate transitions since last update
    uint32_t current_count = m->hall_count;
    uint32_t delta_count = current_count - m->prev_hall_count;
    
    if (delta_count == 0) {
        return m->public_state.measured_rpm;  // Return previous value
    }
    
    // Calculate time delta
    uint64_t delta_time_us = now_us - m->prev_rpm_calc_time_us;
    if (delta_time_us == 0) {
        return m->public_state.measured_rpm;
    }
    
    // RPM calculation:
    // Hall sensors produce 6 transitions per electrical revolution
    // For P pole pairs: 6*P transitions per mechanical revolution
    // RPM = (transitions / (6 * pole_pairs)) * (60_000_000 us/min / time_us)
    
    float revolutions = (float)delta_count / (6.0f * m->pole_pairs);
    float minutes = (float)delta_time_us / 60000000.0f;
    float rpm = revolutions / minutes;
    
    // Update for next calculation
    m->prev_hall_count = current_count;
    m->prev_rpm_calc_time_us = now_us;
    
    return rpm;
}

static float pid_compute(motor_id_t motor, float target_rpm) {
    motor_internal_t* m = &g_motors[motor];
    uint64_t now_us = esp_timer_get_time();
    
    // Calculate dt in seconds
    float dt = (m->pid.last_update_us > 0) ? 
               (now_us - m->pid.last_update_us) / 1000000.0f : 
               (PID_LOOP_PERIOD_MS / 1000.0f);
    m->pid.last_update_us = now_us;
    
    // Get current RPM
    float measured_rpm = calculate_rpm(motor);
    m->public_state.measured_rpm = measured_rpm;
    
    // Calculate error
    float error = target_rpm - measured_rpm;
    
    // Proportional term
    float p_term = m->pid.kp * error;
    
    // Integral term with anti-windup
    m->pid.integral += error * dt;
    m->pid.integral = fmaxf(-m->pid.integral_max, fminf(m->pid.integral_max, m->pid.integral));
    float i_term = m->pid.ki * m->pid.integral;
    
    // Derivative term
    float d_term = m->pid.kd * (error - m->pid.prev_error) / dt;
    m->pid.prev_error = error;
    
    // Compute output
    float output = p_term + i_term + d_term;
    
    // Clamp output
    output = fmaxf(m->pid.output_min, fminf(m->pid.output_max, output));
    
    return output;
}

static void set_throttle_voltage(motor_id_t motor, float voltage) {
    motor_internal_t* m = &g_motors[motor];
    
    // Clamp voltage to safe range
    voltage = fmaxf(0.0f, fminf(3.3f, voltage));
    
    // Convert voltage to DAC value (0-255 for 0-3.3V)
    uint8_t dac_value = (uint8_t)(voltage * 255.0f / 3.3f);
    
    // Output to DAC
    dac_oneshot_output_voltage(m->dac_handle, dac_value);
    
    // Update state
    m->public_state.throttle_voltage = voltage;
}

static float apply_deadzone(float value, float deadzone) {
    if (fabsf(value) < deadzone) {
        return 0.0f;
    }
    // Scale remaining range
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * (fabsf(value) - deadzone) / (1.0f - deadzone);
}

static void pid_control_task(void* pvParameters) {
    ESP_LOGI(TAG, "PID control task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PID_LOOP_PERIOD_MS);
    
    while (1) {
        // Wait for next period
        vTaskDelayUntil(&last_wake_time, period);
        
        // Get current input
        float speed = g_current_input.speed;
        float turn = g_current_input.turn;
        
        // Differential drive calculation
        // Left motor: speed - turn
        // Right motor: speed + turn
        float target_left = speed - turn * 0.5f;   // 0.5 = turn sensitivity
        float target_right = speed + turn * 0.5f;
        
        // Clamp to [-1.0, 1.0]
        target_left = fmaxf(-1.0f, fminf(1.0f, target_left));
        target_right = fmaxf(-1.0f, fminf(1.0f, target_right));
        
        // Convert to target RPM (assuming max_rpm from config, or use fixed value)
        const float MAX_RPM = 4500.0f;  // Updated to match your motors' max RPM
        float target_rpm_left = target_left * MAX_RPM;
        float target_rpm_right = target_right * MAX_RPM;
        
        // Update target RPM in state
        g_motors[MOTOR_LEFT].public_state.target_rpm = target_rpm_left;
        g_motors[MOTOR_RIGHT].public_state.target_rpm = target_rpm_right;
        
        // Run PID for each motor
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (!g_motors[i].public_state.enabled) {
                set_throttle_voltage(i, 0.0f);
                continue;
            }
            
            float target_rpm = (i == MOTOR_LEFT) ? target_rpm_left : target_rpm_right;
            
            // PID compute
            float output_voltage = pid_compute(i, target_rpm);
            
            // Set throttle
            set_throttle_voltage(i, output_voltage);
        }
    }
}
