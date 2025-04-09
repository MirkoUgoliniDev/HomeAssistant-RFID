#ifndef _MQTT_MANAGER_H_
#define _MQTT_MANAGER_H_

#include <esp_err.h>
#include "config.h"

// Stati del client MQTT
typedef enum {
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_manager_state_t;




void mqtt_publish_availability(void *arg);


void start_availability_task(const char *device_id);



esp_err_t mqtt_manager_send_discovery(const app_config_t *config);


void send_rfid_uid(const char *uid);




// Callback per i messaggi MQTT ricevuti
typedef void (*mqtt_message_callback_t)(const char* topic, const char* data, int data_len);

// Callback per i cambiamenti di stato
typedef void (*mqtt_state_callback_t)(mqtt_manager_state_t state);

/**
 * @brief Inizializza il manager MQTT
 * @param config Configurazione con i parametri MQTT
 * @param message_cb Callback per i messaggi ricevuti
 * @param state_cb Callback per i cambiamenti di stato
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_init(const app_config_t *config, mqtt_message_callback_t message_cb,mqtt_state_callback_t state_cb);

/**
 * @brief Avvia la connessione MQTT
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_start(void);

/**
 * @brief Ferma la connessione MQTT
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_stop(void);

/**
 * @brief Pubblica un messaggio su un topic
 * @param topic Topic su cui pubblicare
 * @param data Dati da pubblicare
 * @param len Lunghezza dei dati
 * @param qos Livello QoS (0,1,2)
 * @param retain Flag retain
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_publish(const char* topic, const void* data, int len, int qos, int retain);

/**
 * @brief Sottoscrive ad un topic
 * @param topic Topic a cui sottoscriversi
 * @param qos Livello QoS (0,1,2)
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_subscribe(const char* topic, int qos);

/**
 * @brief Cancella la sottoscrizione da un topic
 * @param topic Topic da cui cancellare la sottoscrizione
 * @return ESP_OK se successo
 */
esp_err_t mqtt_manager_unsubscribe(const char* topic);

/**
 * @brief Ottiene lo stato attuale del client MQTT
 * @return Stato attuale
 */
mqtt_manager_state_t mqtt_manager_get_state(void);

/**
 * @brief Verifica se il client MQTT è connesso
 * @return true se connesso
 */
bool mqtt_manager_is_connected(void);

#endif // _MQTT_MANAGER_H_