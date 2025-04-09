#include "rfid_manager.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#include "esp_log.h"
#include <string.h>
#include "mqtt_manager.h"
#include "led_manager.h"


static const char *TAG = "rfid_manager";



// Variabili statiche
static rc522_driver_handle_t driver = NULL;
static rc522_handle_t scanner = NULL;
static rfid_card_callback_t card_callback = NULL;
static rfid_removal_callback_t removal_callback = NULL;
static bool card_present = false;



// Handler per gli eventi del PICC
static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;



    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        rfid_uid_t uid = {0};
        
        // Copia l'UID
        uid.length = picc->uid.length;
        memcpy(uid.bytes, picc->uid.value, picc->uid.length);
        
        // Aggiorna lo stato
        card_present = true;
        
        // Log dell'UID
        char uid_str[32];


        rfid_uid_to_string(&uid, uid_str, sizeof(uid_str));


        ESP_LOGI(TAG, "Card detected. UID: %s", uid_str);



        /*
        TODO: Da modificare
        led_set_color(255, 0, 0); // Rosso
        vTaskDelay(pdMS_TO_TICKS(1000));

        led_set_color(0, 255, 0); // Verde
        vTaskDelay(pdMS_TO_TICKS(1000));

        led_set_color(0, 0, 255); // Blu
        vTaskDelay(pdMS_TO_TICKS(1000));
        */


        if (!mqtt_manager_is_connected()) {

            ESP_LOGW(TAG, "MQTT not connected, cannot publish!");

        } else {

            mqtt_manager_publish(g_config.mqtt_rfid_topic_uid, uid_str, strlen(uid_str), 0, 0);

        }


        // Notifica attraverso il callback
        if (card_callback) {
            card_callback(&uid);
        }

    }else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {

        // Aggiorna lo stato
        card_present = false;
        
        ESP_LOGI(TAG, "Card removed");
        
        // Notifica attraverso il callback
        if (removal_callback) {
            removal_callback();
        }

    }

}

esp_err_t rfid_manager_init(rfid_card_callback_t card_cb, rfid_removal_callback_t removal_cb)
{
    // Salva i callback
    card_callback = card_cb;
    removal_callback = removal_cb;

    // Configura SPI
    static spi_bus_config_t bus_config = {
        .miso_io_num = CONFIG_RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = CONFIG_RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = CONFIG_RC522_SPI_BUS_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    // Configura il driver RC522
    static rc522_spi_config_t driver_config = {
        .host_id = SPI3_HOST,
        .bus_config = &bus_config,
        .dev_config = {
            .spics_io_num = CONFIG_RC522_SPI_BUS_GPIO_SDA,
        },
        .rst_io_num = CONFIG_RC522_SPI_BUS_GPIO_RST,
    };

    // Crea e installa il driver
    esp_err_t ret = rc522_spi_create(&driver_config, &driver);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RC522 driver");
        return ret;
    }

    ret = rc522_driver_install(driver);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install RC522 driver");
        return ret;
    }

    // Configura lo scanner
    rc522_config_t scanner_config = {
        .driver = driver,
    };

    ret = rc522_create(&scanner_config, &scanner);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RC522 scanner");
        return ret;
    }

    // Registra il callback per gli eventi
    ret = rc522_register_events(scanner, 
                               RC522_EVENT_PICC_STATE_CHANGED, 
                               on_picc_state_changed, 
                               NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register RC522 events");
        return ret;
    }

    ESP_LOGI(TAG, "RFID Manager initialized successfully");
    return ESP_OK;
}

esp_err_t rfid_manager_start(void)
{
    if (scanner == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = rc522_start(scanner);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RFID scanner started");
    } else {
        ESP_LOGE(TAG, "Failed to start RFID scanner");
    }
    return ret;
}



/*
esp_err_t rfid_manager_stop(void)
{
    if (scanner == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    
    esp_err_t ret = rc522_stop(scanner);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RFID scanner stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop RFID scanner");
    }
    return ret;
    
}
*/


esp_err_t rfid_manager_stop(void)
{
    if (scanner == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "RFID scanner stop requested, but no stop function available");
    return ESP_OK;
}





esp_err_t rfid_uid_to_string(const rfid_uid_t *uid, char *str, size_t str_size)
{
    if (uid == NULL || str == NULL || str_size < (uid->length * 2 + 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < uid->length; i++) {
        snprintf(str + (i * 2), 3, "%02X", uid->bytes[i]);
    }

    return ESP_OK;
}

bool rfid_is_card_present(void)
{
    return card_present;
}