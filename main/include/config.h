#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <esp_err.h>
#include <stdbool.h>



 /**
  * @struct app_config_t
  * @brief Structure to hold the application configuration
  */
 typedef struct {
    bool is_configured;               /*!< Flag indicating if the device is configured */
    char device_id[24];               /*!< Device identifier */
    char wifi_ssid[24];               /*!< WiFi SSID */
    char wifi_password[32];           /*!< WiFi password */
    char mqtt_broker[64];             /*!< MQTT broker URI */
    char mqtt_username[24];           /*!< MQTT username */
    char mqtt_password[24];           /*!< MQTT password */
    char mqtt_rfid_topic_uid[64];     /*!< MQTT topic for RFID UID */
    char mqtt_alarm_topic_status[64]; /*!< MQTT topic for alarm status */
    char mqtt_alarm_topic_set[64];    /*!< MQTT topic for setting alarm */
    char mqtt_alarm_topic_get[64];    /*!< MQTT topic for getting alarm status */
    bool mqtt_enable_availability;
} app_config_t;



/**
 * @brief Inizializza la configurazione con i valori predefiniti
 * @param config Puntatore alla struttura di configurazione da inizializzare
 */
void init_default_config(app_config_t *config);


// External declaration of the global config
extern app_config_t g_config;

// Funzioni per la gestione della configurazione
void config_reset(void);
bool config_is_valid(void);
esp_err_t config_save(const app_config_t *config);
esp_err_t config_load(app_config_t *config);
esp_err_t config_init(void);


// Dichiarazione della funzione callback per la nuova configurazione
void config_received_callback(const app_config_t *new_config);


#endif // _CONFIG_H_