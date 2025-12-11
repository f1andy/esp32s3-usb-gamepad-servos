/*
 * Minimal USB HID gamepad -> servo/relay controller
 * Target: ESP32-S3, ESP-IDF v5.5.1
 *
 * - USB Host HID for a generic gamepad (e.g. Switch Pro, VID=0x057e, PID=0x2009)
 * - Left stick: BASE (X) and SHOULDER (Y)
 * - Right stick: ELBOW (X) and JAW (Y)
 * - Buttons: A/B -> MOTOR1 on/off, X/Y -> MOTOR2 on/off
 *
HID BUTTON MAP (Genesis Mangan 300 Wired, HID Mode)

Byte 0:
  bit0 = X
  bit1 = A
  bit2 = B
  bit3 = Y
  bit4 = L1
  bit5 = R1
  bit6 = L2
  bit7 = R2

Byte 1:
  bit0 = Minus (-)
  bit1 = Plus (+)
  bit4 = H (Home)
  T and O buttons do not affect any bits

Byte 2 (D-Pad / HAT):
  0x88 = Neutral
  0x00 = Up
  0x44 = Down
  0x66 = Left
  0x22 = Right

Joystick Axes:
  Byte 3 = LX (0=left, FF=right)
  Byte 4 = LY (0=up,   FF=down)
  Byte 5 = RX (0=left, FF=right)
  Byte 6 = RY (0=up,   FF=down)
*/


#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_err.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

// Optional: table of protocol names, like Espressif example
static const char *hid_proto_name_str[] = {
    "NONE",     // 0
    "KEYBOARD", // HID_PROTOCOL_KEYBOARD
    "MOUSE",    // HID_PROTOCOL_MOUSE
    "JOYSTICK", // if your version defines it, otherwise ignore
};
static void process_gamepad_report(const uint8_t *data, size_t len);

/* -----------------------
 * Application pins
 * ----------------------*/

// Servos
#define SERVO_BASE_GPIO GPIO_NUM_4
#define SERVO_ELBOW_GPIO GPIO_NUM_5
#define SERVO_SHOULDER_GPIO GPIO_NUM_6
#define SERVO_JAW_GPIO GPIO_NUM_7

// Relays
#define MOTOR1_GPIO GPIO_NUM_17
#define MOTOR2_GPIO GPIO_NUM_18

// Sensors
#define SENSOR1_GPIO GPIO_NUM_41
#define SENSOR2_GPIO GPIO_NUM_42

// LEDC configuration
#define SERVO_LEDC_TIMER LEDC_TIMER_0
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_RES LEDC_TIMER_14_BIT
#define SERVO_LEDC_FREQ_HZ 50

// Gamepad buttons (first byte of report, assumed layout)
#define GAMEPAD_BTN_A (1 << 1)
#define GAMEPAD_BTN_B (1 << 2)
#define GAMEPAD_BTN_X (1 << 0)
#define GAMEPAD_BTN_Y (1 << 3)

/* -----------------------
 * Types / state
 * ----------------------*/


static const char *TAG = "usb_gamepad";

typedef enum
{
    SERVO_BASE = 0,
    SERVO_ELBOW,
    SERVO_SHOULDER,
    SERVO_JAW,
    SERVO_COUNT
} servo_id_t;

typedef struct
{
    gpio_num_t gpio;
    ledc_channel_t channel;
    float center_us;     // Center pulse width in microseconds
    float travel_us;     // +/- travel from center
    float deadband_norm; // Deadband around 0.0 in normalized units (-1..1)
} servo_cfg_t;

static servo_cfg_t s_servos[SERVO_COUNT] = {
    {SERVO_BASE_GPIO, LEDC_CHANNEL_0, 1500.0f, 500.0f, 0.05f},
    {SERVO_ELBOW_GPIO, LEDC_CHANNEL_1, 1500.0f, 500.0f, 0.05f},
    {SERVO_SHOULDER_GPIO, LEDC_CHANNEL_2, 1500.0f, 500.0f, 0.05f},
    {SERVO_JAW_GPIO, LEDC_CHANNEL_3, 1500.0f, 500.0f, 0.05f},
};

/* Relay state */
static bool s_motor1_on = false;
static bool s_motor2_on = false;

/* -----------------------
 * Servo helpers
 * ----------------------*/

static void servo_ledc_init(void)
{
    // Configure common LEDC timer for servos
    ledc_timer_config_t timer = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_RES,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Configure channels
    for (int i = 0; i < SERVO_COUNT; ++i)
    {
        ledc_channel_config_t ch = {
            .speed_mode = SERVO_LEDC_MODE,
            .channel = s_servos[i].channel,
            .timer_sel = SERVO_LEDC_TIMER,
            .gpio_num = s_servos[i].gpio,
            .duty = 0,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
}

/* Convert pulse width in microseconds to LEDC duty */
static uint32_t servo_pulse_to_duty(float pulse_us)
{
    const uint32_t max_duty = (1 << SERVO_LEDC_RES) - 1;            // 2^14 - 1
    const float period_us = 1000000.0f / (float)SERVO_LEDC_FREQ_HZ; // e.g. 20ms
    float duty = (pulse_us / period_us) * (float)max_duty;

    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > (float)max_duty)
        duty = (float)max_duty;

    return (uint32_t)duty;
}

/* Set servo from normalized value in range [-1.0, 1.0] */
static void servo_set_normalized(servo_id_t id, float norm)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        return;
    }

    servo_cfg_t *cfg = &s_servos[id];

    // Apply deadband
    if (fabsf(norm) < cfg->deadband_norm)
    {
        norm = 0.0f;
    }

    if (norm > 1.0f)
        norm = 1.0f;
    if (norm < -1.0f)
        norm = -1.0f;

    float pulse = cfg->center_us + norm * cfg->travel_us;

    // Clamp to [center - travel, center + travel]
    float min_pulse = cfg->center_us - cfg->travel_us;
    float max_pulse = cfg->center_us + cfg->travel_us;
    if (pulse < min_pulse)
        pulse = min_pulse;
    if (pulse > max_pulse)
        pulse = max_pulse;

    uint32_t duty = servo_pulse_to_duty(pulse);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, cfg->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, cfg->channel));
}

/* Helper to map 0..255 axis to -1..1 */
static float axis_u8_to_norm(uint8_t v)
{
    // Many controllers use 0x80 as center
    const float center = 128.0f;
    return ( (float)v - center ) / center;
}

/* -----------------------
 * Relay & sensor helpers
 * ----------------------*/

static void io_init(void)
{
    // Relays as outputs
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << MOTOR1_GPIO) | (1ULL << MOTOR2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    // Ensure both motors off
    gpio_set_level(MOTOR1_GPIO, 0);
    gpio_set_level(MOTOR2_GPIO, 0);

    // Sensors as inputs (with pull-ups)
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << SENSOR1_GPIO) | (1ULL << SENSOR2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));
}


static void motor_update_outputs(void)
{
    gpio_set_level(MOTOR1_GPIO, s_motor1_on ? 1 : 0);
    gpio_set_level(MOTOR2_GPIO, s_motor2_on ? 1 : 0);
}

/* -----------------------
 * USB Host / HID Host
 * ----------------------*/


/*
 * Background task that drives the generic USB Host library.
 * HID Host runs its own background task (configured below).
 */
static void usb_host_event_task(void *arg)
{
    uint32_t event_flags;

    while (1)
    {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "usb_host_lib_handle_events failed: %s", esp_err_to_name(err));
        }
        // We don't handle NO_CLIENTS / ALL_FREE events here; app runs forever.
    }
}

static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;

    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event)
    {

    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        // Read raw HID input report
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
            hid_device_handle,
            data,
            sizeof(data),
            &data_length));
                  process_gamepad_report(data, data_length); 
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
                 hid_proto_name_str[dev_params.proto]);
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
                 hid_proto_name_str[dev_params.proto]);
        break;

    default:
        ESP_LOGW(TAG, "HID Device, protocol '%s' UNHANDLED interface event %d",
                 hid_proto_name_str[dev_params.proto], event);
        break;
    }
}
/*
 * Called by HID Host when a HID device appears (driver-level event).
 */
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event)
    {

    case HID_HOST_DRIVER_EVENT_CONNECTED:
    {

        ESP_LOGI(TAG, "HID Device CONNECTED, proto=%d sub_class=%d",
                 dev_params.proto, dev_params.sub_class);

        // Continue with HID interface setup
        const hid_host_device_config_t dev_cfg = {
            .callback = hid_host_interface_callback,
            .callback_arg = NULL};

        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_cfg));
        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;
    }

    default:
        ESP_LOGW(TAG, "Unhandled HID driver event: %d", event);
        break;
    }
}

/*
 * Very simple gamepad report parser.
 
 */

static void process_gamepad_report(const uint8_t *r, size_t len)
{
    if (len < 7) {
        // Report must contain at least buttons + D-pad + 4 axes
        return;
    }

    /* ----------------------------------------------------
     * HID Report Layout (8 bytes):
     * r[0] = Buttons (A,B,X,Y,L1,R1,L2,R2)
     * r[1] = System buttons (-, +, Home)
     * r[2] = D-pad (0x88 neutral)
     * r[3] = LX (0–255)
     * r[4] = LY (0–255)
     * r[5] = RX (0–255)
     * r[6] = RY (0–255)
     * r[7] = unused/reserved
     * ---------------------------------------------------- */

    uint8_t buttons = r[0];

    uint8_t lx = r[3];
    uint8_t ly = r[4];
    uint8_t rx = r[5];
    uint8_t ry = r[6];

    /* ----------------------------------------------------
     * Servo mapping (normalized −1.0 → +1.0)
     * ---------------------------------------------------- */

    servo_set_normalized(SERVO_BASE,     axis_u8_to_norm(lx));
    servo_set_normalized(SERVO_SHOULDER, axis_u8_to_norm(ly));
    servo_set_normalized(SERVO_ELBOW,    axis_u8_to_norm(rx));
    servo_set_normalized(SERVO_JAW,      axis_u8_to_norm(ry));

    /* ----------------------------------------------------
     * Button→Relay mapping
     * Your button map:
     * bit0 = X
     * bit1 = A
     * bit2 = B
     * bit3 = Y
     * ---------------------------------------------------- */

    bool btnX = buttons & 0x01;
    bool btnA = buttons & 0x02;
    bool btnB = buttons & 0x04;
    bool btnY = buttons & 0x08;

    // MOTOR1: A=start, B=stop
    if (btnA) s_motor1_on = true;
    if (btnB) s_motor1_on = false;

    // MOTOR2: X=start, Y=stop
    if (btnX) s_motor2_on = true;
    if (btnY) s_motor2_on = false;

    motor_update_outputs();
}

/* -----------------------
 * app_main
 * ----------------------*/

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 USB gamepad servo controller starting...");

    // Init IO
    io_init();
    servo_ledc_init();

    // Center all servos at startup
    for (int i = 0; i < SERVO_COUNT; ++i)
    {
        servo_set_normalized((servo_id_t)i, 0.0f);
    }

    // USB Host library configuration
    const usb_host_config_t host_cfg = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    // Start USB host event task
    BaseType_t created = xTaskCreatePinnedToCore(
        usb_host_event_task,
        "usb_host_events",
        4096,
        NULL,
        5,
        NULL,
        0);
    if (created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create usb_host_event_task");
        return;
    }

    // Configure HID Host driver
    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true, // HID driver runs its own task
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_event,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    ESP_LOGI(TAG, "Waiting for USB gamepad (e.g. Switch Pro, VID=0x057e, PID=0x2009)");
    ESP_LOGI(TAG, "Left stick: BASE/SHOULDER, Right stick: ELBOW/JAW, Buttons A/B/X/Y -> MOTOR1/MOTOR2");

    // app_main can return or optionally monitor sensors in a loop.
    // We'll just periodically log sensor states.
    while (1)
    {
        int s1 = gpio_get_level(SENSOR1_GPIO);
        int s2 = gpio_get_level(SENSOR2_GPIO);
        ESP_LOGD(TAG, "SENSOR1=%d SENSOR2=%d", s1, s2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
