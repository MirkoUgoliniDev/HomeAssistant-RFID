#ifndef _WIFI_MANAGER_H_
#define _WIFI_MANAGER_H_

#include <esp_err.h>
#include "config.h"

// Enumeration per gli stati del WiFi
typedef enum {
    WIFI_STATE_NOT_INITIALIZED,
    WIFI_STATE_AP_STARTED,        // Access Point mode
    WIFI_STATE_STA_DISCONNECTED,  // Station mode disconnesso
    WIFI_STATE_STA_CONNECTING,    // Station mode in connessione
    WIFI_STATE_STA_CONNECTED      // Station mode connesso
} wifi_manager_state_t;

// Callback per notificare i cambiamenti di stato
typedef void (*wifi_manager_callback_t)(wifi_manager_state_t state);

/**
 * @brief Inizializza il WiFi Manager
 * @param callback Funzione di callback per le notifiche di stato
 * @return ESP_OK se successo
 */
esp_err_t wifi_manager_init(wifi_manager_callback_t callback);

/**
 * @brief Avvia il WiFi in modalità Access Point per la configurazione
 * @return ESP_OK se successo
 */
esp_err_t wifi_manager_start_ap(void);

/**
 * @brief Avvia il WiFi in modalità Station usando i parametri configurati
 * @param config Configurazione contenente SSID e password
 * @return ESP_OK se successo
 */
esp_err_t wifi_manager_start_sta(const app_config_t *config);

/**
 * @brief Ferma il WiFi (sia AP che STA mode)
 * @return ESP_OK se successo
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief Ottiene lo stato attuale del WiFi
 * @return Stato attuale del WiFi
 */
wifi_manager_state_t wifi_manager_get_state(void);

/**
 * @brief Verifica se il WiFi è connesso in modalità station
 * @return true se connesso
 */
bool wifi_manager_is_connected(void);

#endif // _WIFI_MANAGER_H_