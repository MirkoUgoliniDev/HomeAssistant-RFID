/**
 * @file dfplayer_example.c
 * @brief Example of using DFPlayer Mini with ESP-IDF
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "DFPlayerMiniFast.h"

static const char *TAG = "dfplayer_example";

// UART settings
#define DFPLAYER_UART_PORT UART_NUM_2
#define DFPLAYER_UART_TXD  17
#define DFPLAYER_UART_RXD  16
#define DFPLAYER_UART_BAUD 9600

// Buffer sizes
#define DFPLAYER_UART_BUF_SIZE 128

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing DFPlayer Mini example");
    
    // Configure UART for DFPlayer
    uart_config_t uart_config = {
        .baud_rate = DFPLAYER_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(DFPLAYER_UART_PORT, DFPLAYER_UART_BUF_SIZE * 2, 
                                       DFPLAYER_UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DFPLAYER_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DFPLAYER_UART_PORT, DFPLAYER_UART_TXD, DFPLAYER_UART_RXD, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialize DFPlayer
    dfplayer_config_t dfplayer_config = {
        .uart_port = DFPLAYER_UART_PORT,
        .debug = true,
        .timeout_ms = 1000,
    };
    
    dfplayer_handle_t dfplayer = dfplayer_init(&dfplayer_config);
    if (dfplayer == NULL) {
        ESP_LOGE(TAG, "Failed to initialize DFPlayer");
        return;
    }
    
    // Wait for DFPlayer to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Set volume to max
    ESP_LOGI(TAG, "Setting volume to max");
    ESP_ERROR_CHECK(dfplayer_volume(dfplayer, 30));
    
    // Get current volume
    int16_t volume;
    if (dfplayer_get_volume(dfplayer, &volume) == ESP_OK) {
        ESP_LOGI(TAG, "Current volume: %d", volume);
    }
    
    // Loop track 1
    ESP_LOGI(TAG, "Looping track 1");
    ESP_ERROR_CHECK(dfplayer_loop(dfplayer, 1));
    
    // Main loop
    while (1) {
        // Get play status
        bool is_playing;
        if (dfplayer_is_playing(dfplayer, &is_playing) == ESP_OK) {
            ESP_LOGI(TAG, "Playing status: %s", is_playing ? "playing" : "not playing");
        }
        
        // Wait 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}