#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

// Dichiarazione delle variabili globali per i topic
extern char mqtt_rfid_topic_uid[100];
extern char mqtt_alarm_topic_status[100];
extern char mqtt_alarm_topic_set[100];
extern char mqtt_alarm_topic_get[100];

// Dichiarazione della funzione di inizializzazione
void init_mqtt_topics();

#endif // MQTT_TOPICS_H