#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "PHOTODIODE_TEST"
#define PHOTODIODE_CHANNEL ADC_CHANNEL_0        // GPIO-0
#define ADC_UNIT_USED      ADC_UNIT_1
#define ADC_ATTEN_USED     ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH      ADC_BITWIDTH_12      // 12 is the most detailed, can adjust based on board
#define THRESHOLD          32 			// Adjust this based on photodiode readings, and the environment
#define DOT_DURATION_MS    20   		// Must match sender's UNIT (50 ms)
#define MAX_MORSE_BUFFER   16

char morseBuffer[MAX_MORSE_BUFFER];
int morseIndex = 0;

char decodeMorse(const char *code) {
    struct { const char *morse; char letter; } table[] = {
        {".-", 'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'}, {".", 'E'},
        {"..-.", 'F'}, {"--.", 'G'}, {"....", 'H'}, {"..", 'I'}, {".---", 'J'},
        {"-.-", 'K'}, {".-..", 'L'}, {"--", 'M'}, {"-.", 'N'}, {"---", 'O'},
        {".--.", 'P'}, {"--.-", 'Q'}, {".-.", 'R'}, {"...", 'S'}, {"-", 'T'},
        {"..-", 'U'}, {"...-", 'V'}, {".--", 'W'}, {"-..-", 'X'}, {"-.--", 'Y'},
        {"--..", 'Z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'},
        {"...--", '3'}, {"....-", '4'}, {".....", '5'}, {"-....", '6'},
        {"--...", '7'}, {"---..", '8'}, {"----.", '9'}
    };

    for (int i = 0; i < sizeof(table) / sizeof(*table); i++) {
        if (strcmp(code, table[i].morse) == 0) {
            return table[i].letter;
        }
    }
    return '?';
}

void app_main(void) {
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_USED,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, PHOTODIODE_CHANNEL, &chan_config));

    adc_cali_handle_t cali_handle = NULL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_USED,
        .chan = PHOTODIODE_CHANNEL,
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BIT_WIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK) {
        calibrated = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting enabled");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_USED,
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BIT_WIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle) == ESP_OK) {
        calibrated = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting enabled");
    }
#endif

    static bool prevState = false;
    static int64_t lastTransitionTime = 0;

    while (1) {
        int adc_raw = 0;
        int voltage_mv = 0;

        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, PHOTODIODE_CHANNEL, &adc_raw));
        bool ledOn = adc_raw > THRESHOLD;

        if (calibrated) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv));
            // ESP_LOGI(TAG, "ADC Raw: %d, Voltage: %d mV", adc_raw, voltage_mv);
        } else {
            // ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);
        }

        int64_t now = esp_timer_get_time();  // microseconds

        if (ledOn != prevState) {
            int64_t duration_us = now - lastTransitionTime;
            lastTransitionTime = now;

            if (prevState) {
                // LED turned OFF — symbol ended
                if (morseIndex < MAX_MORSE_BUFFER - 1) {
                    if (duration_us < 2 * DOT_DURATION_MS * 1000) {
                        morseBuffer[morseIndex++] = '.';
                    } else {
                        morseBuffer[morseIndex++] = '-';
                    }
                }
            } else {
                // LED turned ON — gap ended
                if (duration_us >= 6 * DOT_DURATION_MS * 1000) {
                    // Word gap
                    if (morseIndex > 0) {
                        morseBuffer[morseIndex] = '\0';
                        char decoded = decodeMorse(morseBuffer);
                        ESP_LOGI(TAG, "Decoded: %c", decoded);
                        morseIndex = 0;
                    }
                    ESP_LOGI(TAG, "(space)");
                } else if (duration_us >= 2 * DOT_DURATION_MS * 1000) {
                    // Letter gap
                    if (morseIndex > 0) {
                        morseBuffer[morseIndex] = '\0';
                        char decoded = decodeMorse(morseBuffer);
                        ESP_LOGI(TAG, "Decoded: %c", decoded);
                        morseIndex = 0;
                    }
                }
            }

            prevState = ledOn;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // sample every 10 ms
    }
}

