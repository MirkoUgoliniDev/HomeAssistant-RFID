#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"  // Per accedere ai CONFIG_*
#include "mqtt_topics.h" // Header file che definiremo dopo
#include <esp_log.h>
#include <stdbool.h>  // For bool type support


static const char *TAG = "mqtt_topics.c";


// Definizione delle variabili globali per i topic
char mqtt_rfid_topic_uid[100];
char mqtt_alarm_topic_status[100];
char mqtt_alarm_topic_set[100];
char mqtt_alarm_topic_get[100];


// Funzione helper per sostituire il placeholder
// Modificata per ritornare bool invece di void
static bool replace_placeholder(const char* template, const char* placeholder, const char* value, char* result, size_t result_size) {
    char* pos = strstr(template, placeholder);
    if (pos) {
        size_t prefix_len = pos - template;
        snprintf(result, result_size, "%.*s%s%s", (int)prefix_len, template, value, pos + strlen(placeholder));
        return true; // Sostituzione riuscita
    } else {
        strncpy(result, template, result_size);
        return false; // Placeholder non trovato
    }
}


void init_mqtt_topics() {
    
    const char* device_id = CONFIG_DEFAULT_DEVICE_ID;

    ESP_LOGI(TAG, "DEVICE ID: %s", device_id );


    // Verifica preliminare del device_id
    if (!device_id || strlen(device_id) == 0) {
        ESP_LOGE(TAG, "Device ID non valido.");
        return;
    }

    // Effettua la sostituzione dei topic con device_id
    bool res = replace_placeholder(CONFIG_DEFAULT_MQTT_RFID_TOPIC_UID, "{device_id}", device_id, mqtt_rfid_topic_uid, sizeof(mqtt_rfid_topic_uid));
    if (!res) {
        ESP_LOGE(TAG, "Errore nella sostituzione di mqtt_rfid_topic_uid.");
    }

    res = replace_placeholder(CONFIG_DEFAULT_MQTT_ALARM_TOPIC_STATUS, "{device_id}", device_id, mqtt_alarm_topic_status, sizeof(mqtt_alarm_topic_status));
    if (!res) {
        ESP_LOGE(TAG, "Errore nella sostituzione di mqtt_alarm_topic_status.");
    }

    res = replace_placeholder(CONFIG_DEFAULT_MQTT_ALARM_TOPIC_SET, "{device_id}", device_id, mqtt_alarm_topic_set, sizeof(mqtt_alarm_topic_set));
    if (!res) {
        ESP_LOGE(TAG, "Errore nella sostituzione di mqtt_alarm_topic_set.");
    }

    res = replace_placeholder(CONFIG_DEFAULT_MQTT_ALARM_TOPIC_GET, "{device_id}", device_id, mqtt_alarm_topic_get, sizeof(mqtt_alarm_topic_get));
    if (!res) {
        ESP_LOGE(TAG, "Errore nella sostituzione di mqtt_alarm_topic_get.");
    }
}