/**
 * gpio_platform.c
 * 
 * Arduino-like API interface to general-purpose I/O functions for the 
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 * 
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "gpio_platform_esp.h"
 #include "esp_timer.h"
 #include "esp_log.h"
 #include <inttypes.h>
 
 static const char *TAG = "GPIO_PLATFORM";
 
 /**
  * These functions emulate Arduino general pin IO,
  * leaving the original source code intact.
  * 
  */
 

 void pinMode(short pin, short mode){
     // arduino definition: mode = INPUT, OUTPUT, or INPUT_PULLUP, or INPUT_PULLDOWN
     // Added OUTPUT_OPEN for open-drain output
     ESP_LOGI(TAG, "Configuring GPIO pin %d, mode: %d", pin, mode);
 
     gpio_config_t io_conf;
     io_conf.pin_bit_mask = 1ULL << ((gpio_num_t) pin);
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     
     switch(mode) {
         case INPUT:
             io_conf.mode = GPIO_MODE_INPUT;
             ESP_LOGI(TAG, "Pin %d set as INPUT", pin);
             break;
         case INPUT_PULLUP:
             io_conf.mode = GPIO_MODE_INPUT;
             io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
             ESP_LOGI(TAG, "Pin %d set as INPUT with PULLUP", pin);
             break;
         case INPUT_PULLDOWN:
             io_conf.mode = GPIO_MODE_INPUT;
             io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
             ESP_LOGI(TAG, "Pin %d set as INPUT with PULLDOWN", pin);
             break;
         case OUTPUT:
             io_conf.mode = GPIO_MODE_OUTPUT;
             ESP_LOGI(TAG, "Pin %d set as OUTPUT", pin);
             break;
         case OUTPUT_OPEN:
             io_conf.mode = GPIO_MODE_OUTPUT_OD;
             ESP_LOGI(TAG, "Pin %d set as OUTPUT OPEN-DRAIN", pin);
             break;
         default:
             ESP_LOGW(TAG, "Pin %d set with UNKNOWN mode: %d", pin, mode);
             break;
     }
     
     esp_err_t result = gpio_config(&io_conf);
     if (result != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure GPIO pin %d, error: %d", pin, result);
     }
     ESP_ERROR_CHECK(result);
 }
 

 
 void digitalWrite(short pin, short value){
     ESP_LOGD(TAG, "Writing to GPIO pin %d: %d", pin, value);
     gpio_set_level((gpio_num_t) pin, (uint32_t) (value != 0));
 }
 


 int digitalRead(short pin){
     int level = gpio_get_level((gpio_num_t) pin);
     ESP_LOGD(TAG, "Reading from GPIO pin %d: %d", pin, level);
     return level;
 }
 

 void delay(int ms){
     ESP_LOGV(TAG, "Delaying for %d ms", ms);
     vTaskDelay(ms / portTICK_PERIOD_MS);
 }
 

 uint32_t millis(){
     uint32_t ms = (uint32_t) (esp_timer_get_time() / 1000);
     // Convertire esplicitamente a unsigned int per evitare problemi di formato
     ESP_LOGV(TAG, "Current millis: %u", (unsigned int)ms);
     return ms;
 }