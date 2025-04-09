#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include <string.h>
#include "esp_mac.h"
#include "sdkconfig.h"


static const char *TAG = "wifi_manager";


// Variabili statiche per il WiFi Manager
static wifi_manager_state_t wifi_state = WIFI_STATE_NOT_INITIALIZED;
static wifi_manager_callback_t state_callback = NULL;
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;

// Prototipi delle funzioni statiche
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

esp_err_t wifi_manager_init(wifi_manager_callback_t callback) {
    esp_err_t ret = ESP_OK;

    // Inizializza il sottosistema TCP/IP se non è già stato fatto
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Crea il loop di eventi di default se non esiste
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Crea le interfacce di rete di default per AP e STA
    ap_netif = esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();

    // Inizializza il WiFi con la configurazione di default
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Registra gli handler per gli eventi WiFi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_event_handler,NULL,NULL));

    // Salva il callback
    state_callback = callback;
    
    // Aggiorna lo stato
    wifi_state = WIFI_STATE_NOT_INITIALIZED;
    
    ESP_LOGI(TAG, "WiFi manager initialized");
    return ret;
}



esp_err_t wifi_manager_start_ap(void) {
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_AP_SSID,
            .ssid_len = strlen(CONFIG_AP_SSID),
            .password = CONFIG_AP_PASS,
            .max_connection = CONFIG_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = CONFIG_AP_CHANNEL
        },
    };

    if (strlen(CONFIG_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s password:%s channel:%d", CONFIG_AP_SSID, CONFIG_AP_PASS, CONFIG_AP_CHANNEL);

    wifi_state = WIFI_STATE_AP_STARTED;
    if (state_callback) {
        state_callback(wifi_state);
    }

    return ESP_OK;
}

esp_err_t wifi_manager_start_sta(const app_config_t *config) {

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strncpy((char *)wifi_config.sta.ssid, config->wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, config->wifi_password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_state = WIFI_STATE_STA_CONNECTING;
    if (state_callback) {
        state_callback(wifi_state);
    }

    ESP_LOGI(TAG, "Connecting to SSID:%s password:%s",config->wifi_ssid, "********");

    return ESP_OK;
}


esp_err_t wifi_manager_stop(void) {
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) {
        wifi_state = WIFI_STATE_NOT_INITIALIZED;
        if (state_callback) {
            state_callback(wifi_state);
        }
    }
    return err;
}


wifi_manager_state_t wifi_manager_get_state(void) {
    return wifi_state;
}

bool wifi_manager_is_connected(void) {
    return (wifi_state == WIFI_STATE_STA_CONNECTED);
}

// Handler degli eventi WiFi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                wifi_state = WIFI_STATE_STA_CONNECTED;
                if (state_callback) {
                    state_callback(wifi_state);
                }
                ESP_LOGI(TAG, "Connected to AP");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                wifi_state = WIFI_STATE_STA_DISCONNECTED;
                if (state_callback) {
                    state_callback(wifi_state);
                }
                ESP_LOGI(TAG, "Disconnected from AP");
                esp_wifi_connect(); // Riprova a connettersi
                break;
                
                case WIFI_EVENT_AP_STACONNECTED: {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);
                } break;
                
                case WIFI_EVENT_AP_STADISCONNECTED: {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",MAC2STR(event->mac), event->aid);
                } break;

        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}