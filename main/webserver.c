#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <string.h>
#include <stdlib.h>
#include "esp_netif.h"
#include <sys/param.h>
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

static const char *TAG = "webserver";
static httpd_handle_t server = NULL;
static webserver_config_callback_t config_cb = NULL;
static app_config_t *current_config = NULL;
static bool config_is_local = false;



// Aggiungi questa variabile globale all'inizio del file, insieme alle altre variabili globali
static webserver_uri_registered_callback_t uri_registered_cb = NULL;

// Aggiungi questa funzione nel file webserver.c
void webserver_set_uri_registered_callback(webserver_uri_registered_callback_t callback) {
    uri_registered_cb = callback;
}





// HTML template della pagina di configurazione
// Questo è il template HTML migliorato che puoi sostituire nel tuo codice
// HTML template della pagina di configurazione
// Questo è il template HTML migliorato che puoi sostituire nel tuo codice
static const char *config_html_template = "<!DOCTYPE html>\n"
"<html><head><title>ESP32 RFID Configuration</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<style>"
"body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f4f8; color: #333; }"
".container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); }"
".form-group { margin-bottom: 20px; }"
"label { display: block; margin-bottom: 8px; font-weight: bold; color: #2c3e50; }"
"input[type=\"text\"], input[type=\"password\"] { width: 100%%; padding: 12px; border: 1px solid #ddd; border-radius: 6px; box-sizing: border-box; font-size: 16px; transition: border-color 0.3s; }"
"input[type=\"text\"]:focus, input[type=\"password\"]:focus { border-color: #3498db; outline: none; box-shadow: 0 0 0 3px rgba(52, 152, 219, 0.2); }"
"button { background: #27ae60; color: white; padding: 14px; border: none; border-radius: 6px; cursor: pointer; width: 100%%; font-size: 16px; font-weight: bold; transition: background 0.3s; }"
"button:hover { background: #2ecc71; }"
".header { text-align: center; margin-bottom: 30px; padding-bottom: 15px; border-bottom: 2px solid #f0f4f8; }"
".header h2 { margin: 0; color: #2c3e50; font-size: 28px; }"
".section-title { margin-top: 30px; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 1px solid #f0f4f8; color: #2c3e50; font-size: 20px; }"
".checkbox-container { display: flex; align-items: center; }"
".checkbox-container input[type=\"checkbox\"] { margin-right: 10px; width: auto; }"
".checkbox-container label { display: inline; margin-bottom: 0; }"
".form-text { display: block; margin-top: 5px; font-size: 14px; color: #7f8c8d; }"
"@media (max-width: 600px) { .container { padding: 20px; } }"
"</style></head>"
"<body><div class=\"container\">"
"<div class=\"header\"><h2>ESP32 RFID Configuration</h2></div>"
"<form method=\"POST\" action=\"/save\">"
"<div class=\"form-group\"><label>Device ID:</label><input type=\"text\" name=\"device_id\" value=\"%s\" required></div>"
"<div class=\"section-title\">WiFi Settings</div>"
"<div class=\"form-group\"><label>WiFi SSID:</label><input type=\"text\" name=\"wifi_ssid\" value=\"%s\" required></div>"
"<div class=\"form-group\"><label>WiFi Password:</label><input type=\"password\" name=\"wifi_password\" value=\"%s\" required></div>"
"<div class=\"section-title\">MQTT Broker Settings</div>"
"<div class=\"form-group\"><label>MQTT Broker URI:</label><input type=\"text\" name=\"mqtt_broker\" value=\"%s\" required></div>"
"<div class=\"form-group\"><label>MQTT Username:</label><input type=\"text\" name=\"mqtt_username\" value=\"%s\" required></div>"
"<div class=\"form-group\"><label>MQTT Password:</label><input type=\"password\" name=\"mqtt_password\" value=\"%s\" required></div>"
"<div class=\"form-group checkbox-container\">"
"<input type=\"checkbox\" id=\"mqtt_enable_availability\" name=\"mqtt_enable_availability\" %s>"
"<label for=\"mqtt_enable_availability\">Enable MQTT Availability</label>"
"</div>"
"<div class=\"form-text\">Show device status in Home Assistant (online/offline)</div>"
"<div class=\"section-title\">MQTT Topics</div>"
"<div class=\"form-group\"><label>RFID Topic UID:</label><input type=\"text\" name=\"mqtt_rfid_topic_uid\" value=\"%s\"></div>"
"<div class=\"form-group\"><label>Alarm Topic Status:</label><input type=\"text\" name=\"mqtt_alarm_topic_status\" value=\"%s\"></div>"
"<div class=\"form-group\"><label>Alarm Topic Set:</label><input type=\"text\" name=\"mqtt_alarm_topic_set\" value=\"%s\"></div>"
"<div class=\"form-group\"><label>Alarm Topic Get:</label><input type=\"text\" name=\"mqtt_alarm_topic_get\" value=\"%s\"></div>"
"<button type=\"submit\">Save Configuration</button>"
"</form></div></body></html>";




/**
 * @brief Decode URL encoded string
 * @param src URL encoded string
 * @return Decoded string (caller must free)
 */
static char *url_decode(const char *src) {
    char *dest = malloc(strlen(src) + 1);
    if (!dest) return NULL;
    char *dst = dest;
    while (*src) {
        if (*src == '%') {
            if (*(src+1) && *(src+2)) {
                char hex[3] = { *(src+1), *(src+2), '\0' };
                *dst++ = (char)strtol(hex, NULL, 16);
                src += 3;
            } else {
                *dst++ = *src++;
            }
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
    return dest;
}




/**
 * @brief Load configuration from NVS
 * @param config Pointer to configuration structure to populate
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t load_config_from_nvs(app_config_t *config) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Check if device is configured
    uint8_t is_configured = 0;
    err = nvs_get_u8(nvs_handle, "configured", &is_configured);
    if (err != ESP_OK) {
        // If not found, device is not configured
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            config->is_configured = false;
            nvs_close(nvs_handle);
            return ESP_OK;
        }
        
        ESP_LOGE(TAG, "Error reading configured flag: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    config->is_configured = (is_configured == 1);

    if (config->is_configured) {
        // Load all configuration fields
        size_t required_size = 0;

        // For each field, get required size first, then get the string
        err = nvs_get_str(nvs_handle, "device_id", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "device_id", config->device_id, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "wifi_ssid", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "wifi_ssid", config->wifi_ssid, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "wifi_pass", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "wifi_pass", config->wifi_password, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "mqtt_broker", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "mqtt_broker", config->mqtt_broker, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "mqtt_user", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "mqtt_user", config->mqtt_username, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "mqtt_pass", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "mqtt_pass", config->mqtt_password, &required_size);
            if (err != ESP_OK) goto error;
        }

        err = nvs_get_str(nvs_handle, "rfid_topic", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "rfid_topic", config->mqtt_rfid_topic_uid, &required_size);
            if (err != ESP_OK) goto error;
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Use default value if not found
            strncpy(config->mqtt_rfid_topic_uid, CONFIG_DEFAULT_MQTT_RFID_TOPIC_UID, sizeof(config->mqtt_rfid_topic_uid)-1);
        }

        err = nvs_get_str(nvs_handle, "alarm_status", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "alarm_status", config->mqtt_alarm_topic_status, &required_size);
            if (err != ESP_OK) goto error;
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Use default value if not found
            strncpy(config->mqtt_alarm_topic_status, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_STATUS, sizeof(config->mqtt_alarm_topic_status)-1);
        }

        err = nvs_get_str(nvs_handle, "alarm_set", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "alarm_set", config->mqtt_alarm_topic_set, &required_size);
            if (err != ESP_OK) goto error;
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Use default value if not found
            strncpy(config->mqtt_alarm_topic_set, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_SET, sizeof(config->mqtt_alarm_topic_set)-1);
        }

        err = nvs_get_str(nvs_handle, "alarm_get", NULL, &required_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs_handle, "alarm_get", config->mqtt_alarm_topic_get, &required_size);
            if (err != ESP_OK) goto error;
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Use default value if not found
            strncpy(config->mqtt_alarm_topic_get, CONFIG_DEFAULT_MQTT_ALARM_TOPIC_GET, sizeof(config->mqtt_alarm_topic_get)-1);
        }
    }

    nvs_close(nvs_handle);
    return ESP_OK;

error:
    ESP_LOGE(TAG, "Error loading configuration: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
}




/**
 * @brief Generate HTML with the current configuration values
 * @return Dynamically allocated HTML string (caller must free)
 */
static char *generate_config_html() {
    // Valori predefiniti da Kconfig.projbuild
    char default_device_id[24] = "";
    char default_wifi_ssid[24] = "";
    char default_wifi_password[32] = "";
    char default_mqtt_broker[64] = "";
    char default_mqtt_username[24] = "";
    char default_mqtt_password[24] = "";
    char default_rfid_topic[64] = CONFIG_DEFAULT_MQTT_RFID_TOPIC_UID;
    char default_alarm_status_topic[64] = CONFIG_DEFAULT_MQTT_ALARM_TOPIC_STATUS;
    char default_alarm_set_topic[64] = CONFIG_DEFAULT_MQTT_ALARM_TOPIC_SET;
    char default_alarm_get_topic[64] = CONFIG_DEFAULT_MQTT_ALARM_TOPIC_GET;
    bool default_mqtt_enable_availability = CONFIG_MQTT_ENABLE_AVAILABILITY;
    char mqtt_availability_checked[10] = "";
    
    // Se c'è una configurazione esistente, usa quei valori invece dei predefiniti
    if (current_config && current_config->is_configured) {
        strncpy(default_device_id, current_config->device_id, sizeof(default_device_id)-1);
        strncpy(default_wifi_ssid, current_config->wifi_ssid, sizeof(default_wifi_ssid)-1);
        strncpy(default_wifi_password, current_config->wifi_password, sizeof(default_wifi_password)-1);
        strncpy(default_mqtt_broker, current_config->mqtt_broker, sizeof(default_mqtt_broker)-1);
        strncpy(default_mqtt_username, current_config->mqtt_username, sizeof(default_mqtt_username)-1);
        strncpy(default_mqtt_password, current_config->mqtt_password, sizeof(default_mqtt_password)-1);
        
        if (strlen(current_config->mqtt_rfid_topic_uid) > 0) {
            strncpy(default_rfid_topic, current_config->mqtt_rfid_topic_uid, sizeof(default_rfid_topic)-1);
        }
        if (strlen(current_config->mqtt_alarm_topic_status) > 0) {
            strncpy(default_alarm_status_topic, current_config->mqtt_alarm_topic_status, sizeof(default_alarm_status_topic)-1);
        }
        if (strlen(current_config->mqtt_alarm_topic_set) > 0) {
            strncpy(default_alarm_set_topic, current_config->mqtt_alarm_topic_set, sizeof(default_alarm_set_topic)-1);
        }
        if (strlen(current_config->mqtt_alarm_topic_get) > 0) {
            strncpy(default_alarm_get_topic, current_config->mqtt_alarm_topic_get, sizeof(default_alarm_get_topic)-1);
        }
        
        default_mqtt_enable_availability = current_config->mqtt_enable_availability;
    }
    
    // Prepara il valore "checked" per il checkbox
    if (default_mqtt_enable_availability) {
        strcpy(mqtt_availability_checked, "checked");
    } else {
        mqtt_availability_checked[0] = '\0';
    }
    
    // Calcola la dimensione necessaria per l'HTML completo
    int html_size = strlen(config_html_template) + 
                    strlen(default_device_id) + 
                    strlen(default_wifi_ssid) + 
                    strlen(default_wifi_password) +
                    strlen(default_mqtt_broker) +
                    strlen(default_mqtt_username) +
                    strlen(default_mqtt_password) +
                    strlen(mqtt_availability_checked) +
                    strlen(default_rfid_topic) +
                    strlen(default_alarm_status_topic) +
                    strlen(default_alarm_set_topic) +
                    strlen(default_alarm_get_topic) +
                    100; // Margine di sicurezza
    
    char *html = malloc(html_size);
    if (!html) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML");
        return NULL;
    }
    
    // Formatta l'HTML con i valori correnti
    snprintf(html, html_size, config_html_template,
             default_device_id,
             default_wifi_ssid,
             default_wifi_password,
             default_mqtt_broker,
             default_mqtt_username,
             default_mqtt_password,
             mqtt_availability_checked,  // Nuovo parametro per il checkbox
             default_rfid_topic,
             default_alarm_status_topic,
             default_alarm_set_topic,
             default_alarm_get_topic);
    
    return html;
}





/**
 * @brief Handle HTTP GET request for configuration page
 */
static esp_err_t config_get_handler(httpd_req_t *req) {
    char *html = generate_config_html();
    if (!html) {
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "text/html");
    esp_err_t res = httpd_resp_send(req, html, strlen(html));
    free(html);
    return res;
}




/**
 * @brief Handle HTTP POST request to save configuration
 */
static esp_err_t config_post_handler(httpd_req_t *req) {
    char content[1024];
    size_t recv_size = MIN(req->content_len, sizeof(content)-1);
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive POST data");
        return ESP_FAIL;
    }
    content[recv_size] = '\0';

    app_config_t new_config = {0};

    char *saveptr;
    char *param = strtok_r(content, "&", &saveptr);

    while (param) {
        char *value = strchr(param, '=');
        if (value) {
            *value = '\0';
            value++;
            char *decoded_value = url_decode(value);

            if (decoded_value) {
                if (strcmp(param, "device_id") == 0) {
                    strncpy(new_config.device_id, decoded_value, sizeof(new_config.device_id)-1);
                } else if (strcmp(param, "wifi_ssid") == 0) {
                    strncpy(new_config.wifi_ssid, decoded_value, sizeof(new_config.wifi_ssid)-1);
                } else if (strcmp(param, "wifi_password") == 0) {
                    strncpy(new_config.wifi_password, decoded_value, sizeof(new_config.wifi_password)-1);
                } else if (strcmp(param, "mqtt_broker") == 0) {
                    strncpy(new_config.mqtt_broker, decoded_value, sizeof(new_config.mqtt_broker)-1);
                } else if (strcmp(param, "mqtt_username") == 0) {
                    strncpy(new_config.mqtt_username, decoded_value, sizeof(new_config.mqtt_username)-1);
                } else if (strcmp(param, "mqtt_password") == 0) {
                    strncpy(new_config.mqtt_password, decoded_value, sizeof(new_config.mqtt_password)-1);
                } else if (strcmp(param, "mqtt_rfid_topic_uid") == 0) {
                    strncpy(new_config.mqtt_rfid_topic_uid, decoded_value, sizeof(new_config.mqtt_rfid_topic_uid)-1);
                } else if (strcmp(param, "mqtt_alarm_topic_status") == 0) {
                    strncpy(new_config.mqtt_alarm_topic_status, decoded_value, sizeof(new_config.mqtt_alarm_topic_status)-1);
                } else if (strcmp(param, "mqtt_alarm_topic_set") == 0) {
                    strncpy(new_config.mqtt_alarm_topic_set, decoded_value, sizeof(new_config.mqtt_alarm_topic_set)-1);
                } else if (strcmp(param, "mqtt_alarm_topic_get") == 0) {
                    strncpy(new_config.mqtt_alarm_topic_get, decoded_value, sizeof(new_config.mqtt_alarm_topic_get)-1);
                }
                free(decoded_value);
            }
        }
        param = strtok_r(NULL, "&", &saveptr);
    }

    new_config.is_configured = true;

    // Salva la configurazione in NVS
    esp_err_t err = config_save(&new_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration to NVS");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        return ESP_FAIL;
    }

    // Aggiorna la configurazione corrente
    if (current_config) {
        memcpy(current_config, &new_config, sizeof(app_config_t));
    }

    // Chiama la callback se registrata
    if (config_cb) {
        config_cb(&new_config);
    }

    // Invia una pagina HTML di conferma invece di un semplice messaggio di testo
    const char *success_html = "<!DOCTYPE html>\n"
        "<html><head><title>Configuration Saved</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<style>"
        "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f4f8; color: #333; text-align: center; }"
        ".container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); }"
        ".success-icon { font-size: 64px; color: #27ae60; margin-bottom: 20px; }"
        "h2 { color: #2c3e50; margin-bottom: 20px; }"
        "p { font-size: 16px; line-height: 1.5; margin-bottom: 30px; }"
        ".countdown { font-weight: bold; color: #e74c3c; }"
        "</style>"
        "<script>"
        "let seconds = 15;"
        "function updateCountdown() {"
        "  document.getElementById('countdown').textContent = seconds;"
        "  if (seconds <= 0) return;"
        "  seconds--;"
        "  setTimeout(updateCountdown, 1000);"
        "}"
        "window.onload = function() {"
        "  updateCountdown();"
        "  setTimeout(function() {"
        "    document.getElementById('message').textContent = 'Riavvio in corso...';"
        "  }, 5000);"
        "};"
        "</script></head>"
        "<body><div class=\"container\">"
        "<div class=\"success-icon\">✓</div>"
        "<h2>Configurazione Salvata!</h2>"
        "<p>Il dispositivo si riavvierà tra <span id=\"countdown\" class=\"countdown\">5</span> secondi.</p>"
        "<p id=\"message\">Dopo il riavvio, il dispositivo si connetterà alla rete WiFi configurata.</p>"
        "</div></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, success_html, strlen(success_html));
}








/**
 * @brief Check if the web server is running
 */
bool webserver_is_running(void) {
    return (server != NULL);
}

/**
 * @brief Get the URL of the web server
 */
const char* webserver_get_url(void) {
    if (!webserver_is_running()) {
        return NULL;
    }
    
    // In a real implementation, you would return the IP address or hostname
    // of the server, but this would depend on your network configuration
    return "http://esp32.local";
}



/**
 * @brief Start the web server
 */
esp_err_t webserver_start(webserver_config_callback_t config_callback, app_config_t *config) {
    if (server != NULL) {
        ESP_LOGE(TAG, "Server already started");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Se non è stato fornito un puntatore alla configurazione, creiamo una copia locale
    if (config == NULL) {
        current_config = malloc(sizeof(app_config_t));
        if (current_config == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for configuration");
            return ESP_ERR_NO_MEM;
        }
        memset(current_config, 0, sizeof(app_config_t));
        config_is_local = true;
        
        // Carica la configurazione da NVS
        esp_err_t err = load_config_from_nvs(current_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load configuration from NVS");
            free(current_config);
            current_config = NULL;
            return err;
        }
    } else {
        current_config = config;
        config_is_local = false;
    }
    
    config_cb = config_callback;
    
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.stack_size = 8192; // Aumenta stack size per gestire HTML più grande
    ESP_LOGI(TAG, "Starting server");
    
    if (httpd_start(&server, &server_config) == ESP_OK) {
        httpd_uri_t get_config_page = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = config_get_handler,
            .user_ctx = NULL
        };


        //httpd_register_uri_handler(server, &get_config_page);

        if (httpd_register_uri_handler(server, &get_config_page) == ESP_OK) {
            if (uri_registered_cb) {
                uri_registered_cb(get_config_page.uri, get_config_page.method);
            }
        }
         


        httpd_uri_t post_config = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = config_post_handler,
            .user_ctx = NULL
        };

        //httpd_register_uri_handler(server, &post_config);
        
        if (httpd_register_uri_handler(server, &post_config) == ESP_OK) {
            if (uri_registered_cb) {
                uri_registered_cb(post_config.uri, post_config.method);
            }
        }



        ESP_LOGI(TAG, "Server started successfully");

        return ESP_OK;

    }
    


    ESP_LOGE(TAG, "Failed to start server");
    if (config_is_local && current_config != NULL) {
        free(current_config);
        current_config = NULL;
    }
    return ESP_FAIL;
}




/**
 * @brief Stop the web server
 */
esp_err_t webserver_stop(void) {
    if (server == NULL) {
        ESP_LOGE(TAG, "Server not started");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Stopping server");
    esp_err_t err = httpd_stop(server);
    server = NULL;
    
    // Libera la memoria se abbiamo creato una copia locale della configurazione
    if (config_is_local && current_config != NULL) {
        free(current_config);
        current_config = NULL;
    }
    
    return err;
}




/**
 * @brief Restituisce l'handle del server HTTP attivo
 * 
 * Questa funzione permette ai componenti esterni (come il captive portal)
 * di accedere all'handle del server HTTP per registrare i propri handler.
 * 
 * @return httpd_handle_t Handle del server HTTP se attivo, NULL altrimenti
 */
httpd_handle_t webserver_get_server_handle(void) {
    return server;
}