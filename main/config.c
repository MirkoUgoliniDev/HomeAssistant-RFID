#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include "mqtt_topics.h"


static const char *TAG = "config";

// Variabile globale di configurazione
app_config_t g_config;

// Chiavi NVS
#define NVS_NAMESPACE                 "storage"
#define NVS_KEY_IS_CONFIGURED          "is_config"
#define NVS_KEY_MQTT_BROKER            "mqtt_broker"
#define NVS_KEY_MQTT_USER              "mqtt_user"
#define NVS_KEY_MQTT_PASS              "mqtt_pass"
#define NVS_KEY_MQTT_RFID              "mqtt_rfid"
#define NVS_KEY_MQTT_ALARM_STATUS      "mqtt_alm_stat"     
#define NVS_KEY_MQTT_ALARM_SET         "mqtt_alm_set"      
#define NVS_KEY_MQTT_ALARM_GET         "mqtt_alm_get"      
#define NVS_KEY_WIFI_SSID              "wifi_ssid"
#define NVS_KEY_WIFI_PASS              "wifi_pass"
#define NVS_KEY_DEVICE_ID              "device_id"


// **Inizializza NVS**
esp_err_t config_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrotto, eseguo erase...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}




/**
 * @brief Inizializza la configurazione con i valori predefiniti
 * @param config Puntatore alla struttura di configurazione da inizializzare
 */
void init_default_config(app_config_t *config) {
    if (!config) return;
    
    memset(config, 0, sizeof(app_config_t));
    
    // Imposta i valori predefiniti presi da sdkconfig.h
    config->is_configured = false;
    
    // Imposta tutti i campi con i valori predefiniti da CONFIG_*
    strncpy(config->device_id, CONFIG_DEFAULT_DEVICE_ID, sizeof(config->device_id)-1);
    strncpy(config->wifi_ssid, CONFIG_DEFAULT_WIFI_SSID, sizeof(config->wifi_ssid)-1);
    strncpy(config->wifi_password, CONFIG_DEFAULT_WIFI_PASSWORD, sizeof(config->wifi_password)-1);
    strncpy(config->mqtt_broker, CONFIG_DEFAULT_MQTT_BROKER, sizeof(config->mqtt_broker)-1);
    strncpy(config->mqtt_username, CONFIG_DEFAULT_MQTT_USERNAME, sizeof(config->mqtt_username)-1);
    strncpy(config->mqtt_password, CONFIG_DEFAULT_MQTT_PASSWORD, sizeof(config->mqtt_password)-1);
    
    // Imposta i valori predefiniti per i topic MQTT dai CONFIG_* definiti in sdkconfig.h
    strncpy(config->mqtt_rfid_topic_uid, CONFIG_DEFAULT_MQTT_RFID_TOPIC_UID, sizeof(config->mqtt_rfid_topic_uid)-1);
    strncpy(config->mqtt_alarm_topic_status, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_STATUS, sizeof(config->mqtt_alarm_topic_status)-1);
    strncpy(config->mqtt_alarm_topic_set, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_SET, sizeof(config->mqtt_alarm_topic_set)-1);
    strncpy(config->mqtt_alarm_topic_get, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_GET, sizeof(config->mqtt_alarm_topic_get)-1);

    // Imposta il valore predefinito per mqtt_enable_availability
    config->mqtt_enable_availability = CONFIG_MQTT_ENABLE_AVAILABILITY;
   
}




// **Reset della configurazione**
void config_reset(void) {
    // Inizializza con valori predefiniti
    init_default_config(&g_config);
    
    ESP_LOGI(TAG, "Configuration reset to defaults");
    
    // Salva in NVS
    config_save(&g_config);
}







// **Verifica validità configurazione**
bool config_is_valid(void) {
    return (strlen(g_config.wifi_ssid) > 0 && strlen(g_config.wifi_password) > 0 &&
            strlen(g_config.mqtt_broker) > 0 && strlen(g_config.mqtt_username) > 0 &&
            strlen(g_config.mqtt_password) > 0);
}



// **Salva configurazione in NVS**
esp_err_t config_save(const app_config_t *config) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore apertura NVS: %s", esp_err_to_name(err));
        return err;
    }

    err |= nvs_set_str(nvs, NVS_KEY_WIFI_SSID, config->wifi_ssid);
    err |= nvs_set_str(nvs, NVS_KEY_WIFI_PASS, config->wifi_password);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_BROKER, config->mqtt_broker);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_USER, config->mqtt_username);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_PASS, config->mqtt_password);
    err |= nvs_set_str(nvs, NVS_KEY_DEVICE_ID, config->device_id);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_RFID, config->mqtt_rfid_topic_uid);

    err |= nvs_set_str(nvs, NVS_KEY_MQTT_ALARM_STATUS, config->mqtt_alarm_topic_status);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_ALARM_SET, config->mqtt_alarm_topic_set);
    err |= nvs_set_str(nvs, NVS_KEY_MQTT_ALARM_GET, config->mqtt_alarm_topic_get);

    err |= nvs_set_u8(nvs, NVS_KEY_IS_CONFIGURED, config->is_configured);

    err |= nvs_commit(nvs);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore salvataggio configurazione NVS: %s", esp_err_to_name(err));
    }
    return err;
}




/**
 * Carica la configurazione dell'applicazione dalla memoria non volatile (NVS)
 * 
 * @param config Puntatore alla struttura di configurazione da popolare
 * @return ESP_OK se la configurazione è stata caricata con successo, altrimenti un codice di errore
 */
esp_err_t config_load(app_config_t *config) {
    // Apre il namespace NVS in modalità sola lettura
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore apertura NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Verifica se il dispositivo è già stato configurato
    uint8_t is_configured = 0;
    err = nvs_get_u8(nvs, NVS_KEY_IS_CONFIGURED, &is_configured);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }

    // Se il dispositivo non è configurato, esce con errore
    if (!is_configured) {
        nvs_close(nvs);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    // Definizione della macro per caricare stringhe da NVS
    size_t required_size;
    #define LOAD_NVS_STRING(key, dest) \
        if (nvs_get_str(nvs, key, NULL, &required_size) == ESP_OK) \
            nvs_get_str(nvs, key, dest, &required_size);

    // Carica le configurazioni WiFi
    LOAD_NVS_STRING(NVS_KEY_WIFI_SSID, config->wifi_ssid);
    LOAD_NVS_STRING(NVS_KEY_WIFI_PASS, config->wifi_password);
    
    // Carica le configurazioni MQTT broker
    LOAD_NVS_STRING(NVS_KEY_MQTT_BROKER, config->mqtt_broker);
    LOAD_NVS_STRING(NVS_KEY_MQTT_USER, config->mqtt_username);
    LOAD_NVS_STRING(NVS_KEY_MQTT_PASS, config->mqtt_password);
    
    // Carica l'ID del dispositivo
    LOAD_NVS_STRING(NVS_KEY_DEVICE_ID, config->device_id);
    
    // Carica i topic MQTT
    LOAD_NVS_STRING(NVS_KEY_MQTT_RFID, config->mqtt_rfid_topic_uid);
    LOAD_NVS_STRING(NVS_KEY_MQTT_ALARM_STATUS, config->mqtt_alarm_topic_status);
    LOAD_NVS_STRING(NVS_KEY_MQTT_ALARM_SET, config->mqtt_alarm_topic_set);
    LOAD_NVS_STRING(NVS_KEY_MQTT_ALARM_GET, config->mqtt_alarm_topic_get);


    // Imposta il flag di configurazione
    config->is_configured = is_configured;
    
    // Chiude l'handle NVS
    nvs_close(nvs);

    // Imposta valori predefiniti per i topic MQTT se sono vuoti
    if (strlen(config->mqtt_rfid_topic_uid) == 0) {
        strncpy(config->mqtt_rfid_topic_uid, CONFIG_DEFAULT_MQTT_RFID_TOPIC_UID, sizeof(config->mqtt_rfid_topic_uid) - 1);
    }
    if (strlen(config->mqtt_alarm_topic_status) == 0) {
        strncpy(config->mqtt_alarm_topic_status, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_STATUS, sizeof(config->mqtt_alarm_topic_status) - 1);
    }
    if (strlen(config->mqtt_alarm_topic_set) == 0) {
        strncpy(config->mqtt_alarm_topic_set, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_SET, sizeof(config->mqtt_alarm_topic_set) - 1);
    }
    if (strlen(config->mqtt_alarm_topic_get) == 0) {
        strncpy(config->mqtt_alarm_topic_get, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_GET, sizeof(config->mqtt_alarm_topic_get) - 1);
    }


    // Logga la configurazione MQTT per debug
    ESP_LOGI(TAG, "Configurazione MQTT:");
    ESP_LOGI(TAG, "RFID_TOPIC: %s", config->mqtt_rfid_topic_uid);
    ESP_LOGI(TAG, "ALARM_STATUS: %s", config->mqtt_alarm_topic_status);
    ESP_LOGI(TAG, "ALARM_SET: %s", config->mqtt_alarm_topic_set);
    ESP_LOGI(TAG, "ALARM_GET: %s", config->mqtt_alarm_topic_get);

    return ESP_OK;
}