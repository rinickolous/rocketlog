#include "freertos/FreeRTOS.h"

#include "led_strip.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

// Sample rate of 1Hz is the maximum sample rate of the MPL3115A2 barometer.
static const TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000);

static const char *TAG = "transmitter";

#define RGB_LED_GPIO ROCKETLOG_RGB_LED_GPIO

static led_strip_handle_t led_strip;

/* ---------------------------------------- */
