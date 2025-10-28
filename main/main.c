// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help

#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/dac_oneshot.h>
#include <esp_log.h>

#include <uni.h>

// LED Configuration
#define LED1_PIN GPIO_NUM_2          // Built-in LED on most ESP32 boards
#define LED_LEDC_TIMER LEDC_TIMER_0
#define LED_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LED1_LEDC_CHANNEL LEDC_CHANNEL_0
#define LED_LEDC_DUTY_RES LEDC_TIMER_8_BIT  // Use 8-bit for more manageable values
#define LED_LEDC_FREQUENCY 5000     // 5 kHz
#define LED_MAX_DUTY (1 << LED_LEDC_DUTY_RES) - 1  // 255 for 8-bit

// LED Control State (only LED1 for blinking)
static TaskHandle_t led_blink_task_handle = NULL;
static bool led1_is_blinking = false;
static uint32_t led1_brightness = LED_MAX_DUTY / 2;  // Start at 50% brightness

// DAC Configuration
static dac_oneshot_handle_t dac_chan0_handle;
static dac_oneshot_handle_t dac_chan1_handle;
static bool dac_test_mode = true;  // Set to true for test mode, false for joystick control
static TaskHandle_t dac_test_task_handle = NULL;


// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} my_platform_instance_t;

// LED Control Functions
static void led_blink_task(void* pvParameters);
static void led_init(void);
static void led1_set_brightness(uint32_t brightness);
static void led1_start_blinking(void);
static void led1_stop_blinking(void);
static void dac_chan0_set_voltage(uint8_t value);
static void dac_chan1_set_voltage(uint8_t value);
static void dac_test_sweep_task(void* pvParameters);

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

//
// Platform Overrides
//
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("custom: init()\n");
    
    // Initialize LED control
    led_init();

#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;

    uni_gamepad_set_mappings(&mappings);
#endif
    //    uni_bt_service_set_enabled(true);
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");

    // Safe to call "unsafe" functions since they are called from BT thread

    // Start scanning
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);

    // Keep stored BT keys for permanent pairing - don't delete them
    // This allows the controller to reconnect automatically
    uni_bt_list_keys_unsafe();  // Just list the keys, don't delete
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here.
    // Just return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.

    // As an example, if you want to filter out keyboards, do:
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev = {0};
    uni_gamepad_t* gp;

    // Optimization to avoid processing the previous data so that the console
    // does not get spammed with a lot of logs, but remove it from your project.
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            gp = &ctl->gamepad;

            // Debug: Print joystick values for debugging
            static int debug_counter = 0;
            if (debug_counter++ % 50 == 0) {  // Print every 50th update to avoid spam
                logi("Joysticks - Left X/Y: %d/%d, Right X/Y: %d/%d\n", 
                     gp->axis_x, gp->axis_y, gp->axis_rx, gp->axis_ry);
            }

            // BUTTON_A: Exit test mode when pressed
            static bool prev_button_a = false;
            bool current_button_a = (gp->buttons & BUTTON_A) != 0;
            if (current_button_a && !prev_button_a && dac_test_mode) {
                dac_test_mode = false;
                logi("***** EXITING DAC TEST MODE *****\n");
                logi("Now in JOYSTICK CONTROL mode\n");
                logi("Left joystick -> GPIO25 (DAC_CHAN_0)\n");
                logi("Right joystick -> GPIO26 (DAC_CHAN_1)\n");
            }
            prev_button_a = current_button_a;

            // Only process joystick input if NOT in test mode
            if (!dac_test_mode) {
                // LED1 Control: Toggle blinking with X button (PS4 Square button)
                static bool prev_button_x = false;
                bool current_button_x = (gp->buttons & BUTTON_X) != 0;
                if (current_button_x && !prev_button_x) {
                    if (led1_is_blinking) {
                        led1_stop_blinking();
                    } else {
                        led1_start_blinking();
                    }
                }
                prev_button_x = current_button_x;

                // DAC Channel 0 (GPIO25): Controlled by left joystick Y-axis
                int32_t left_joystick_y = -gp->axis_y;  // Invert Y-axis (up = higher voltage)
                if (left_joystick_y > 512) left_joystick_y = 512;
                if (left_joystick_y < -512) left_joystick_y = -512;
                int32_t left_normalized = left_joystick_y + 512;  // 0-1024 range
                uint8_t dac0_value = (uint8_t)((left_normalized * LED_MAX_DUTY) / 1024);
                
                static uint8_t prev_dac0_value = LED_MAX_DUTY / 2;
                if (abs((int)dac0_value - (int)prev_dac0_value) > 5) {
                    dac_chan0_set_voltage(dac0_value);
                    led1_set_brightness(dac0_value);  // Also update LED1 brightness
                    prev_dac0_value = dac0_value;
                    float voltage0 = (dac0_value / 255.0f) * 3.3f;
                    logi("DAC0 (GPIO25): %.3fV (value: %d, left Y: %d)\n", voltage0, dac0_value, gp->axis_y);
                }

                // DAC Channel 1 (GPIO26): Controlled by right joystick Y-axis
                int32_t right_joystick_y = -gp->axis_ry;  // Invert Y-axis (up = higher voltage)
                if (right_joystick_y > 512) right_joystick_y = 512;
                if (right_joystick_y < -512) right_joystick_y = -512;
                int32_t right_normalized = right_joystick_y + 512;  // 0-1024 range
                uint8_t dac1_value = (uint8_t)((right_normalized * LED_MAX_DUTY) / 1024);
                
                static uint8_t prev_dac1_value = LED_MAX_DUTY / 2;
                if (abs((int)dac1_value - (int)prev_dac1_value) > 5) {
                    dac_chan1_set_voltage(dac1_value);
                    prev_dac1_value = dac1_value;
                    float voltage1 = (dac1_value / 255.0f) * 3.3f;
                    logi("DAC1 (GPIO26): %.3fV (value: %d, right Y: %d)\n", voltage1, dac1_value, gp->axis_ry);
                }
            }
            break;
        default:
            // Handle other controller classes
            break;
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = data;

            if (d == NULL) {
                loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
                return;
            }
            logi("custom: on_device_oob_event(): %d\n", event);

            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

            trigger_event_on_gamepad(d);
            break;
        }

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

//
// LED Control Implementation
//
static void led_init(void) {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LED_LEDC_DUTY_RES,
        .freq_hz = LED_LEDC_FREQUENCY,
        .speed_mode = LED_LEDC_MODE,
        .timer_num = LED_LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure LED1 channel (only one LED for blinking)
    ledc_channel_config_t ledc_channel1 = {
        .channel = LED1_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = LED1_PIN,
        .speed_mode = LED_LEDC_MODE,
        .timer_sel = LED_LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel1);
    
    logi("LED1 initialized on GPIO %d with PWM support\n", LED1_PIN);
    logi("LED_MAX_DUTY = %lu (8-bit = %d)\n", LED_MAX_DUTY, (1 << 8) - 1);

    // Initialize DAC oneshot for GPIO25 (DAC_CHAN_0) and GPIO26 (DAC_CHAN_1)
    dac_oneshot_config_t dac_cfg0 = {
        .chan_id = DAC_CHAN_0,  // GPIO25
    };
    dac_oneshot_config_t dac_cfg1 = {
        .chan_id = DAC_CHAN_1,  // GPIO26
    };
    
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_cfg0, &dac_chan0_handle));
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_cfg1, &dac_chan1_handle));
    
    logi("DAC oneshot initialized on GPIO25 (DAC_CHAN_0) and GPIO26 (DAC_CHAN_1)\n");
    logi("DAC output range: 0-255 (0.0V - 3.3V)\n");
    
    // Start test sweep if in test mode
    if (dac_test_mode) {
        logi("***** DAC TEST MODE ENABLED *****\n");
        logi("Starting voltage sweep on BOTH DAC channels\n");
        logi("Connect multimeter: RED to GPIO25 or GPIO26, BLACK to GND\n");
        logi("Expected voltage range: 0.0V to 3.3V\n");
        logi("Press BUTTON_A (X/Cross) on controller to exit test mode\n");
        xTaskCreate(dac_test_sweep_task, "dac_test_sweep", 4096, NULL, 5, &dac_test_task_handle);
    }
}

// DAC Test Sweep Task - sweeps voltage from 0V to 3.3V on BOTH channels
static void dac_test_sweep_task(void* pvParameters) {
    uint8_t dac_value = 0;
    bool ascending = true;
    
    logi("DAC Test Sweep Started on BOTH channels\n");
    logi("========================================================\n");
    logi("DAC Value | GPIO25 (V) | GPIO26 (V) | Description\n");
    logi("========================================================\n");
    
    while (dac_test_mode) {
        // Output the SAME value to both DAC channels for testing
        dac_oneshot_output_voltage(dac_chan0_handle, dac_value);
        dac_oneshot_output_voltage(dac_chan1_handle, dac_value);
        
        // Calculate and display expected voltage
        float voltage = (dac_value / 255.0f) * 3.3f;
        
        // Log every 32 steps (8 data points in full sweep)
        if (dac_value % 32 == 0) {
            logi("   %3d    |   %.3f V  |   %.3f V  | %s\n", 
                 dac_value, 
                 voltage,
                 voltage,
                 dac_value == 0 ? "MIN" : (dac_value == 255 ? "MAX" : ""));
        }
        
        // Sweep up and down
        if (ascending) {
            if (dac_value == 255) {
                ascending = false;
                logi("========================================================\n");
                logi("Reached MAX voltage (3.3V), sweeping down...\n");
                vTaskDelay(pdMS_TO_TICKS(2000));  // Pause at max
            } else {
                dac_value++;
            }
        } else {
            if (dac_value == 0) {
                ascending = true;
                logi("========================================================\n");
                logi("Reached MIN voltage (0.0V), sweeping up...\n");
                vTaskDelay(pdMS_TO_TICKS(2000));  // Pause at min
            } else {
                dac_value--;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms per step = ~10 seconds for full sweep
    }
    
    logi("DAC Test Mode Exited\n");
    logi("Switching to joystick control mode\n");
    dac_test_task_handle = NULL;
    vTaskDelete(NULL);
}

// Set LED1 brightness (for LED blinking)
static void led1_set_brightness(uint32_t brightness) {
    if (brightness > LED_MAX_DUTY) {
        brightness = LED_MAX_DUTY;
    }
    led1_brightness = brightness;

    // Always update LED1 brightness immediately, whether blinking or not
    if (!led1_is_blinking) {
        ledc_set_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL, brightness);
        ledc_update_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL);
    }
}

// Update DAC channel 0 (GPIO25) based on joystick
static void dac_chan0_set_voltage(uint8_t value) {
    if (!dac_test_mode) {
        dac_oneshot_output_voltage(dac_chan0_handle, value);
    }
}

// Update DAC channel 1 (GPIO26) based on joystick
static void dac_chan1_set_voltage(uint8_t value) {
    if (!dac_test_mode) {
        dac_oneshot_output_voltage(dac_chan1_handle, value);
    }
}


static void led_blink_task(void* pvParameters) {
    bool led_state = false;
    
    while (1) {
        led_state = !led_state;
        
        // Handle LED1 blinking
        if (led1_is_blinking) {
            if (led_state) {
                ledc_set_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL, led1_brightness);
            } else {
                ledc_set_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL, 0);
            }
            ledc_update_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL);
        } else {
            // Keep LED1 off when not blinking
            ledc_set_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL, 0);
            ledc_update_duty(LED_LEDC_MODE, LED1_LEDC_CHANNEL);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Blink every 500ms
    }
}

static void led1_start_blinking(void) {
    if (!led1_is_blinking) {
        led1_is_blinking = true;
        if (led_blink_task_handle == NULL) {
            xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, &led_blink_task_handle);
        }
        uint32_t brightness_percent = (led1_brightness * 100) / LED_MAX_DUTY;
        logi("LED1 blinking started at brightness %lu%%\n", brightness_percent);
    }
}

static void led1_stop_blinking(void) {
    if (led1_is_blinking) {
        led1_is_blinking = false;
        logi("LED1 blinking stopped\n");
    }
}

//
// Helpers
//
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }

    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }

    if (d->report_parser.set_lightbar_color != NULL) {
        uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
        uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
        uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}

//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}
