#include "mqtt_manager.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include <string.h>
#include "mqtt_topics.h"


static const char *TAG = "mqtt_manager";

// Variabili statiche
static esp_mqtt_client_handle_t mqtt_client = NULL;
static mqtt_manager_state_t mqtt_state = MQTT_STATE_DISCONNECTED;
static mqtt_message_callback_t message_callback = NULL;
static mqtt_state_callback_t state_callback = NULL;




void mqtt_publish_availability(void *arg) {
    const char *device_id = (const char *)arg;
    char topic[100];
    snprintf(topic, sizeof(topic), "home/%s/status", device_id);
    
    while (1) {
        if (mqtt_manager_is_connected()) {
            mqtt_manager_publish(topic, "online", strlen("online"), 1, 1);
            ESP_LOGI(TAG, "Pubblicato stato di disponibilità: online su %s", topic);
        }
        vTaskDelay(pdMS_TO_TICKS(60000)); // Pubblica ogni minuto
    }
}


void start_availability_task(const char *device_id) {
    char *device_id_copy = strdup(device_id); // Crea una copia del device_id
    xTaskCreate(mqtt_publish_availability, "mqtt_avail", 2048, (void*)device_id_copy, 5, NULL);
}





esp_err_t mqtt_manager_send_discovery( const app_config_t *config) {
    /*
    https://www.youtube.com/watch?v=VHiCtZqllU8
    https://resinchemtech.blogspot.com/2023/12/mqtt-auto-discovery.html
    https://www.youtube.com/watch?v=5JHKJy21vKA
    */
    if (!mqtt_manager_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *device_id = config->device_id;


    char topic[100];
    char payload[1024]; // Aumentato a 1024 per evitare truncation
    
    // Definisco il topic per la disponibilità
    char availability_topic[100];
    snprintf(availability_topic, sizeof(availability_topic), "home/%s/status", device_id);

    // Discovery per il sensore RFID
    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/config", device_id);
    
if (config->mqtt_enable_availability) {
    // Versione con availability
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\": \"RFID Reader %s\","
        "\"state_topic\": \"%s\","
        "\"unique_id\": \"rfid_reader_%s\","
        "\"icon\": \"mdi:card-bulleted\","
        "\"value_template\": \"{{ value }}\","
        "\"availability_topic\": \"%s\","
        "\"payload_available\": \"online\","
        "\"payload_not_available\": \"offline\","
        "\"device\": {"
        "  \"identifiers\": [\"%s\"],"
        "  \"name\": \"RFID Device %s\","
        "  \"manufacturer\": \"ESP32\","
        "  \"model\": \"RFID Reader\""
        "}"
        "}",
        device_id, mqtt_rfid_topic_uid, device_id, availability_topic, device_id, device_id
    );
} else {
    // Versione senza availability
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\": \"RFID Reader %s\","
        "\"state_topic\": \"%s\","
        "\"unique_id\": \"rfid_reader_%s\","
        "\"icon\": \"mdi:card-bulleted\","
        "\"value_template\": \"{{ value }}\","
        "\"device\": {"
        "  \"identifiers\": [\"%s\"],"
        "  \"name\": \"RFID Device %s\","
        "  \"manufacturer\": \"ESP32\","
        "  \"model\": \"RFID Reader\""
        "}"
        "}",
        device_id, mqtt_rfid_topic_uid, device_id, device_id, device_id
    );
}

    ESP_LOGI(TAG, "Invio Discovery: %s", topic);
    esp_err_t err = mqtt_manager_publish(topic, payload, strlen(payload), 1, 1);
    if (err != ESP_OK) return err;

    // Discovery per lo switch con visualizzazione a levetta orizzontale
    snprintf(topic, sizeof(topic), "homeassistant/switch/%s/config", device_id);
    
    if (config->mqtt_enable_availability) {
    // Versione con availability per lo switch
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\": \"Allarme %s\","
        "\"state_topic\": \"%s\","
        "\"command_topic\": \"%s\","
        "\"payload_on\": \"ARMED\","
        "\"payload_off\": \"DISARMED\","
        "\"state_on\": \"ARMED\","
        "\"state_off\": \"DISARMED\","
        "\"device_class\": \"switch\","
        "\"component\": \"switch\","
        "\"availability_topic\": \"%s\","
        "\"payload_available\": \"online\","
        "\"payload_not_available\": \"offline\","
        "\"unique_id\": \"alarm_switch_%s\","
        "\"device\": {"
        "  \"identifiers\": [\"%s\"],"
        "  \"name\": \"RFID Device %s\","
        "  \"manufacturer\": \"ESP32\","
        "  \"model\": \"RFID Reader\""
        "}"
        "}",
        device_id, mqtt_alarm_topic_status, mqtt_alarm_topic_set, 
        availability_topic, device_id, device_id, device_id
    );
} else {
    // Versione senza availability per lo switch
    snprintf(payload, sizeof(payload),
        "{"
        "\"name\": \"Allarme %s\","
        "\"state_topic\": \"%s\","
        "\"command_topic\": \"%s\","
        "\"payload_on\": \"ARMED\","
        "\"payload_off\": \"DISARMED\","
        "\"state_on\": \"ARMED\","
        "\"state_off\": \"DISARMED\","
        "\"device_class\": \"switch\","
        "\"component\": \"switch\","
        "\"unique_id\": \"alarm_switch_%s\","
        "\"device\": {"
        "  \"identifiers\": [\"%s\"],"
        "  \"name\": \"RFID Device %s\","
        "  \"manufacturer\": \"ESP32\","
        "  \"model\": \"RFID Reader\""
        "}"
        "}",
        device_id, mqtt_alarm_topic_status, mqtt_alarm_topic_set, 
        device_id, device_id, device_id
    );
}

    ESP_LOGI(TAG, "Invio Discovery: %s", topic);
    return mqtt_manager_publish(topic, payload, strlen(payload), 1, 1);
}





// Funzione per inviare l'UID letto su MQTT
void send_rfid_uid(const char *uid) {
    if (!mqtt_manager_is_connected()) {
        ESP_LOGW(TAG, "MQTT non connesso, impossibile inviare UID");
        return;
    }

    esp_err_t err = mqtt_manager_publish(mqtt_rfid_topic_uid, uid, strlen(uid), 0, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "UID %s inviato su %s", uid, mqtt_rfid_topic_uid);
    } else {
        ESP_LOGE(TAG, "Errore nell'invio dell'UID su MQTT");
    }
}


// Funzione per aggiornare lo stato
static void update_state(mqtt_manager_state_t new_state) {
    if (mqtt_state != new_state) {
        mqtt_state = new_state;
        if (state_callback) {
            state_callback(new_state);
        }
    }
}

// Handler degli eventi MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {

        case MQTT_EVENT_CONNECTED:

            ESP_LOGI(TAG, "MQTT Connected");

            // Aggiorno Lo stato
            update_state(MQTT_STATE_CONNECTED);

            // Invio della configurazione Discovery MQTT a Home Assistant
            mqtt_manager_send_discovery(&g_config);

            //Sottoscrizione al topic dello stato dell'allarme
            mqtt_manager_subscribe(mqtt_alarm_topic_status, 0);
            ESP_LOGI(TAG, "Sottoscritto a %s", mqtt_alarm_topic_status);

            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            update_state(MQTT_STATE_DISCONNECTED);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Unsubscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT Published, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
        
            ESP_LOGI(TAG, "MQTT Data: topic=%.*s   data=%.*s", event->topic_len, event->topic,event->data_len, event->data);
            
            if (message_callback) {
                // Creiamo copie terminate da null di topic e data
                char *topic = malloc(event->topic_len + 1);
                char *data = malloc(event->data_len + 1);
                
                if (topic && data) {
                    memcpy(topic, event->topic, event->topic_len);
                    memcpy(data, event->data, event->data_len);
                    topic[event->topic_len] = '\0';
                    data[event->data_len] = '\0';
                    
                    message_callback(topic, data, event->data_len);
                    
                    free(topic);
                    free(data);
                }
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Error");
            update_state(MQTT_STATE_ERROR);
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}




esp_err_t mqtt_manager_init(const app_config_t *config, mqtt_message_callback_t message_cb, mqtt_state_callback_t state_cb) {
    if (mqtt_client != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    message_callback = message_cb;
    state_callback = state_cb;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->mqtt_broker,
        .credentials.username = config->mqtt_username,
        .credentials.authentication.password = config->mqtt_password,
        .network.reconnect_timeout_ms = 10000,
        .session.disable_keepalive = false,
        .session.keepalive = 60,
        .session.disable_clean_session = false,
    };

    if (config->mqtt_enable_availability) {
        // Aggiungi Last Will and Testament solo se l'availability è abilitato
        char lwt_topic[100];
        snprintf(lwt_topic, sizeof(lwt_topic), "home/%s/status", config->device_id);
        mqtt_cfg.session.last_will.topic = lwt_topic;
        mqtt_cfg.session.last_will.msg = "offline";
        mqtt_cfg.session.last_will.msg_len = strlen("offline");
        mqtt_cfg.session.last_will.qos = 1;
        mqtt_cfg.session.last_will.retain = 1;
    }

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return err;
    }

    update_state(MQTT_STATE_DISCONNECTED);
    return ESP_OK;
}





esp_err_t mqtt_manager_start(void) {

    if (mqtt_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    update_state(MQTT_STATE_CONNECTING);
    return esp_mqtt_client_start(mqtt_client);
}


esp_err_t mqtt_manager_stop(void) {
    if (mqtt_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_mqtt_client_stop(mqtt_client);
    if (err == ESP_OK) {
        update_state(MQTT_STATE_DISCONNECTED);
    }
    return err;
}


esp_err_t mqtt_manager_publish(const char* topic, const void* data, int len, int qos, int retain) {

    if (mqtt_client == NULL || mqtt_state != MQTT_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, len, qos, retain);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}


esp_err_t mqtt_manager_subscribe(const char* topic, int qos) {
    if (mqtt_client == NULL || mqtt_state != MQTT_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_client, topic, qos);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_manager_unsubscribe(const char* topic) {
    if (mqtt_client == NULL || mqtt_state != MQTT_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_unsubscribe(mqtt_client, topic);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

mqtt_manager_state_t mqtt_manager_get_state(void) {
    return mqtt_state;
}

bool mqtt_manager_is_connected(void) {
    return mqtt_state == MQTT_STATE_CONNECTED;
}