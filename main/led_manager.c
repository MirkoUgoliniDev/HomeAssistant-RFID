#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led_manager.h"

#define LED_STRIP_GPIO_PIN  18
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "LED_MANAGER";
static led_strip_handle_t led_strip = NULL;  // Inizializza esplicitamente a NULL





void led_init(void) {

    ESP_LOGI(TAG, "Inizializzazione LED...");

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        }
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore inizializzazione LED: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LED strip inizializzata con successo");
        // Crea il task LED
    }

}




void led_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip non inizializzato!");
        return;
    }
    
    ESP_LOGI(TAG, "Impostando colore LED: R=%d, G=%d, B=%d", red, green, blue);
    
    esp_err_t err = led_strip_set_pixel(led_strip, 0, red, green, blue);
    ESP_LOGI(TAG, "set_pixel result: %s", esp_err_to_name(err));
    
    if (err == ESP_OK) {
        err = led_strip_refresh(led_strip);
        ESP_LOGI(TAG, "refresh result: %s", esp_err_to_name(err));
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore aggiornamento LED: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LED aggiornato con successo");
    }
}


void led_blink(uint8_t red, uint8_t green, uint8_t blue, int delay_ms, int times) {
    for (int i = 0; i < times; i++) {
        led_set_color(red, green, blue);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        led_set_color(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}