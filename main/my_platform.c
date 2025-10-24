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
#define LED2_PIN GPIO_NUM_4          // External LED on GPIO 4
#define LED_LEDC_TIMER LEDC_TIMER_0
#define LED_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LED1_LEDC_CHANNEL LEDC_CHANNEL_0
#define LED2_LEDC_CHANNEL LEDC_CHANNEL_1
#define LED_LEDC_DUTY_RES LEDC_TIMER_8_BIT  // Use 8-bit for more manageable values
#define LED_LEDC_FREQUENCY 5000     // 5 kHz
#define LED_MAX_DUTY (1 << LED_LEDC_DUTY_RES) - 1  // 255 for 8-bit

// DAC Configuration
#define THROTTLE_DAC_CHAN DAC_CHAN_0  // GPIO25 is DAC channel 0
#define THROTTLE_MIN_MV 0000U         // 0.0V minimum
#define THROTTLE_MAX_MV 3300U         // 3.3V maximum
#define DAC_MAX_VALUE 255             // 8-bit DAC (0-255)
static dac_oneshot_handle_t dac_handle = NULL;
static TaskHandle_t throttle_test_task_handle = NULL;

// LED Control State
static TaskHandle_t led_blink_task_handle = NULL;
static bool led1_is_blinking = false;
static bool led2_is_blinking = false;
static uint32_t led1_brightness = LED_MAX_DUTY / 2;  // Start at 50% brightness
static uint32_t led2_brightness = LED_MAX_DUTY / 2;  // Start at 50% brightness

// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} my_platform_instance_t;

// LED Control Functions
static void led_blink_task(void* pvParameters);
static void led_init(void);
static void led1_set_brightness(uint32_t brightness);
static void led2_set_brightness(uint32_t brightness);
static void led1_start_blinking(void);
static void led1_stop_blinking(void);
static void led2_start_blinking(void);
static void led2_stop_blinking(void);

// DAC/Throttle Control Functions
static void throttle_dac_init(void);
static void throttle_set_voltage_mv(uint32_t mv);
static void throttle_test_sweep_task(void* pvParameters);

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
    static uint8_t leds = 0;
    static uint8_t enabled = true;
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

            // LED2 Control: Toggle blinking with Circle button
            static bool prev_button_b = false;
            bool current_button_b = (gp->buttons & BUTTON_B) != 0;
            if (current_button_b && !prev_button_b) {
                if (led2_is_blinking) {
                    led2_stop_blinking();
                } else {
                    led2_start_blinking();
                }
            }
            prev_button_b = current_button_b;

            // LED1 Brightness Control: Use left joystick Y-axis
            int32_t left_joystick_y = -gp->axis_y;  // Invert Y-axis (up = brighter)
            if (left_joystick_y > 512) left_joystick_y = 512;
            if (left_joystick_y < -512) left_joystick_y = -512;
            int32_t left_normalized = left_joystick_y + 512;  // 0-1024 range
            uint32_t led1_new_brightness = (uint32_t)((left_normalized * LED_MAX_DUTY) / 1024);
            
            static uint32_t prev_led1_brightness = LED_MAX_DUTY / 2;
            if (abs((int32_t)led1_new_brightness - (int32_t)prev_led1_brightness) > (LED_MAX_DUTY / 50)) {
                led1_set_brightness(led1_new_brightness);
                prev_led1_brightness = led1_new_brightness;
                uint32_t led1_percent = (led1_new_brightness * 100) / LED_MAX_DUTY;
                logi("LED1 brightness: %lu%% (left Y: %d)\n", led1_percent, gp->axis_y);
            }

            // LED2 Brightness Control: Use right joystick Y-axis
            int32_t right_joystick_y = -gp->axis_ry;  // Invert Y-axis (up = brighter)
            if (right_joystick_y > 512) right_joystick_y = 512;
            if (right_joystick_y < -512) right_joystick_y = -512;
            int32_t right_normalized = right_joystick_y + 512;  // 0-1024 range
            uint32_t led2_new_brightness = (uint32_t)((right_normalized * LED_MAX_DUTY) / 1024);
            
            static uint32_t prev_led2_brightness = LED_MAX_DUTY / 2;
            if (abs((int32_t)led2_new_brightness - (int32_t)prev_led2_brightness) > (LED_MAX_DUTY / 50)) {
                led2_set_brightness(led2_new_brightness);
                prev_led2_brightness = led2_new_brightness;
                uint32_t led2_percent = (led2_new_brightness * 100) / LED_MAX_DUTY;
                logi("LED2 brightness: %lu%% (right Y: %d)\n", led2_percent, gp->axis_ry);
            }

            // Original functionality: Rumble with Button B
            if ((gp->buttons & BUTTON_B) && d->report_parser.play_dual_rumble != NULL) {
                d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
                                                  255 /* weak magnitude */, 0 /* strong magnitude */);
            }
            
            // Original functionality: Controller LEDs with Button X
            if ((gp->buttons & BUTTON_X) && d->report_parser.set_player_leds != NULL) {
                d->report_parser.set_player_leds(d, leds++ & 0x0f);
            }

            // Original functionality: Toggle Bluetooth connections
            if ((gp->buttons & BUTTON_SHOULDER_L) && enabled) {
                logi("*** Stop scanning\n");
                uni_bt_stop_scanning_safe();
                enabled = false;
            }
            if ((gp->buttons & BUTTON_SHOULDER_R) && !enabled) {
                logi("*** Start scanning\n");
                uni_bt_start_scanning_and_autoconnect_safe();
                enabled = true;
            }
            break;
        default:
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

    // Configure LED1 channel
    ledc_channel_config_t ledc_channel1 = {
        .channel = LED1_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = LED1_PIN,
        .speed_mode = LED_LEDC_MODE,
        .timer_sel = LED_LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel1);

    // Configure LED2 channel
    ledc_channel_config_t ledc_channel2 = {
        .channel = LED2_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = LED2_PIN,
        .speed_mode = LED_LEDC_MODE,
        .timer_sel = LED_LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel2);
    
    logi("LED1 initialized on GPIO %d, LED2 on GPIO %d with PWM support\n", LED1_PIN, LED2_PIN);
    logi("LED_MAX_DUTY = %lu (8-bit = %d)\n", LED_MAX_DUTY, (1 << 8) - 1);
}

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

static void led2_set_brightness(uint32_t brightness) {
    if (brightness > LED_MAX_DUTY) {
        brightness = LED_MAX_DUTY;
    }
    led2_brightness = brightness;
    
    // Always update LED2 brightness immediately, whether blinking or not
    if (!led2_is_blinking) {
        ledc_set_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL, brightness);
        ledc_update_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL);
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

        // Handle LED2 blinking
        if (led2_is_blinking) {
            if (led_state) {
                ledc_set_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL, led2_brightness);
            } else {
                ledc_set_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL, 0);
            }
            ledc_update_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL);
        } else {
            // Keep LED2 off when not blinking
            ledc_set_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL, 0);
            ledc_update_duty(LED_LEDC_MODE, LED2_LEDC_CHANNEL);
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

static void led2_start_blinking(void) {
    if (!led2_is_blinking) {
        led2_is_blinking = true;
        if (led_blink_task_handle == NULL) {
            xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, &led_blink_task_handle);
        }
        uint32_t brightness_percent = (led2_brightness * 100) / LED_MAX_DUTY;
        logi("LED2 blinking started at brightness %lu%%\n", brightness_percent);
    }
}

static void led2_stop_blinking(void) {
    if (led2_is_blinking) {
        led2_is_blinking = false;
        logi("LED2 blinking stopped\n");
    }
}
//
// Helpers
//

//
// DAC/Throttle Control Implementation
//
static void throttle_dac_init(void) {
    dac_oneshot_config_t dac_cfg = {
        .chan_id = THROTTLE_DAC_CHAN,
    };
    
    esp_err_t err = dac_oneshot_new_channel(&dac_cfg, &dac_handle);
    if (err != ESP_OK) {
        loge("Failed to initialize DAC channel: %s\n", esp_err_to_name(err));
        return;
    }
    
    // Set initial voltage to minimum
    throttle_set_voltage_mv(THROTTLE_MIN_MV);
    logi("DAC throttle initialized on GPIO25 (DAC_CHAN_0)\n");
}

static void throttle_set_voltage_mv(uint32_t mv) {
    // Clamp to valid range
    if (mv < THROTTLE_MIN_MV) mv = THROTTLE_MIN_MV;
    if (mv > THROTTLE_MAX_MV) mv = THROTTLE_MAX_MV;
    
    // Map millivolts to DAC value (0-255)
    // DAC output: Vout = (dac_value / 255) * 3.3V
    uint32_t dac_value = (mv * DAC_MAX_VALUE) / 3300;
    
    if (dac_handle) {
        esp_err_t err = dac_oneshot_output_voltage(dac_handle, (uint8_t)dac_value);
        if (err != ESP_OK) {
            loge("DAC output failed: %s\n", esp_err_to_name(err));
        }
    }
}\

static void throttle_test_sweep_task(void* pvParameters) {
    (void)pvParameters;
    const TickType_t delay = pdMS_TO_TICKS(50);  // 50ms steps
    
    logi("Starting throttle test sweep (1.0V - 3.3V)\n");
    
    while (1) {
        // Ramp up from 0.0V to 3.3V
        for (uint32_t mv = THROTTLE_MIN_MV; mv <= THROTTLE_MAX_MV; mv += 50) {
            throttle_set_voltage_mv(mv);
            logi("Throttle: %lu mV (DAC: %lu)\n", mv, (mv * DAC_MAX_VALUE) / 3300);
            vTaskDelay(delay);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));  // Pause at max
        
        // Ramp down from 3.3V to 0.0V
        for (int32_t mv = THROTTLE_MAX_MV; mv >= (int32_t)THROTTLE_MIN_MV; mv -= 50) {
            throttle_set_voltage_mv((uint32_t)mv);
            logi("Throttle: %ld mV (DAC: %ld)\n", mv, (mv * DAC_MAX_VALUE) / 3300);
            vTaskDelay(delay);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));  // Pause at min
    }
}


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