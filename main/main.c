#include <esp_log.h>
#include <inttypes.h>
#include <nvs_flash.h>
#include "driver/gpio.h"
#include "config.h"
#include "wifi_manager.h"
#include "webserver.h"
#include "captive_portal.h"
#include "mqtt_manager.h"
#include "rfid_manager.h"
#include <string.h>  // per strcmp, strncmp, etc
#include "freertos/FreeRTOS.h"  // per vTaskDelay e pdMS_TO_TICKS
#include "esp_system.h"  // per esp_restart
#include "led_manager.h"
#include "DFPlayerMiniFast.h"
#include "freertos/queue.h"
#include "VL53L1X_api.h"
#include "esp_timer.h"
#include "mqtt_topics.h"
#include "sdkconfig.h"


static const char *TAG = "main";

static bool first_boot = true;
static int64_t last_alarm_notification_time = 0;
bool status_reported =0;

// STATO ALLARME
bool armed = false;


// VL53L1
uint16_t TOF = CONFIG_VL53L1_I2C_ADDRESS; 
int     just_a_number = 17;
int     range_mm = 0;
int32_t measurement_cycle = 0;
int64_t last_measurement = 0;

static QueueHandle_t vl53_queue = NULL;
// VL53L1



// DFPLAYER
void dfplayer_task(void *pvParameter);
static dfplayer_handle_t player = NULL;  
void initialize_dfplayer();

typedef struct {
    int track_number; // Numero della traccia da riprodurre
} dfplayer_command_t;

static QueueHandle_t dfplayer_queue = NULL;
// DFPLAYER 




// LED_RGB_WS2812
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_command_t;

static led_command_t last_led_state ;

static QueueHandle_t led_queue = NULL;
// LED_RGB_WS2812



// Stato dell'applicazione
typedef enum {
    APP_STATE_INIT,
    APP_STATE_CONFIG_MODE,
    APP_STATE_NORMAL_MODE,
    APP_STATE_ERROR
} app_state_t;

static app_state_t app_state = APP_STATE_INIT;
static bool should_restart = false;




// Callback when WiFi state changes
static void wifi_state_callback(wifi_manager_state_t state) {
    switch (state) {

        case WIFI_STATE_AP_STARTED:
            ESP_LOGI(TAG, "WiFi AP started, starting config web server");   
            webserver_start(config_received_callback, &g_config);
            break;

        case WIFI_STATE_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected, starting MQTT");


             // Riproduce traccia 3 solo al boot
             if (first_boot && dfplayer_queue != NULL) {
                dfplayer_command_t cmd = {.track_number = 3};
                xQueueSend(dfplayer_queue, &cmd, portMAX_DELAY);
                ESP_LOGI(TAG, "Play Traccia 3");
            }



             // Attendi che la rete si stabilizzi
             vTaskDelay(pdMS_TO_TICKS(4000)); // Attendi 3 secondi


            mqtt_manager_start();
            
            break;

        default:
            break;
    }
}




void led_control_task(void *pvParameter) {

    led_command_t cmd;
    
    while(1) {
        if(xQueueReceive(led_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "LED Task: Ricevuto comando RGB(%d,%d,%d)", cmd.red, cmd.green, cmd.blue);
            led_set_color(cmd.red, cmd.green, cmd.blue);
            ESP_LOGI(TAG, "LED Task: Comando eseguito");
        }
    }
}


void trim_trailing_whitespace(char *str) {
    int len = strlen(str);
    while (len > 0 && (str[len - 1] == '\r' || str[len - 1] == '\n' || str[len - 1] == ' ')) {
        str[len - 1] = '\0';
        len--;
    }
}



static void mqtt_message_callback(const char* topic, const char* data, int data_len) {

    ESP_LOGI(TAG, "MQTT topic: %s", topic);
    ESP_LOGI(TAG, "MQTT Data received:  %s (len=%d)", data, data_len);
    ESP_LOGI(TAG, "MQTT config: %s", mqtt_rfid_topic_uid);


    ESP_LOG_BUFFER_HEX(TAG, data, data_len);

    // Creiamo una copia del messaggio e rimuoviamo eventuali spazi o caratteri di controllo
    char message[data_len + 1];
    memcpy(message, data, data_len);
    message[data_len] = '\0';
    trim_trailing_whitespace(message);

   
    if ((strcmp(topic, mqtt_alarm_topic_status) == 0)  ) {

        ESP_LOGI(TAG, "-------------->");


        if (dfplayer_queue == NULL) {
            ESP_LOGE(TAG, "MQTT: Coda DFPlayer non inizializzata, ignoro comando");
            return;  // Esci dalla funzione se la coda non è valida
        }
        dfplayer_command_t cmd;

        int64_t current_time = esp_timer_get_time();


        if (strncmp(message, "ARMED", 5) == 0) {

            armed = true;

            // Comando LED
            led_command_t led_cmd = {.red = 255, .green = 0, .blue = 0};
            last_led_state = led_cmd;
            xQueueSend(led_queue, &led_cmd, portMAX_DELAY);

        } else if (strncmp(message, "DISARMED", 8) == 0) {

          armed = false;

          // Comando LED
          led_command_t led_cmd = {.red = 0, .green = 255, .blue = 0};
          last_led_state = led_cmd;
          xQueueSend(led_queue, &led_cmd, portMAX_DELAY);
        }


        if (current_time - last_alarm_notification_time >= CONFIG_ALARM_DEBOUNCE_TIME_US) {

            // È trascorso abbastanza tempo, aggiorna il timestamp e riproduci
            last_alarm_notification_time = current_time;

            if (strncmp(message, "ARMED", 5) == 0) {
                cmd.track_number = 2;
                BaseType_t result = xQueueSend(dfplayer_queue, &cmd, pdMS_TO_TICKS(1000));
                if (result == pdTRUE) {
                    ESP_LOGI(TAG, "MQTT: Comando inviato correttamente alla coda");
                } else {
                    ESP_LOGE(TAG, "MQTT: Errore nell'invio del comando alla coda");
                }

            } else if (strncmp(message, "DISARMED", 8) == 0) {
                cmd.track_number = 1;   
                BaseType_t result = xQueueSend(dfplayer_queue, &cmd, pdMS_TO_TICKS(1000));
                if (result == pdTRUE) {
                    ESP_LOGI(TAG, "MQTT: Comando inviato correttamente alla coda");
                } else {
                    ESP_LOGE(TAG, "MQTT: Errore nell'invio del comando alla coda");
                }
            }


       } else {

         // Non è trascorso abbastanza tempo, ignora la richiesta
         ESP_LOGW(TAG, "MQTT: Ignorata richiesta di riproduzione ravvicinata (debounce %d sec)", CONFIG_ALARM_DEBOUNCE_TIME_US / 1000000);

      }

    }



    if ((strcmp(topic, mqtt_alarm_topic_set) == 0) || (strcmp(topic, mqtt_alarm_topic_get) == 0)) {
    
        if (strncmp(message, "ARMED", 5) == 0) {
            
            // Pubblica lo stato aggiornato usando la variabile globale
            ESP_LOGI(TAG, "Pubblicando stato ARMED su topic %s", mqtt_alarm_topic_status);
            esp_err_t pub_result = mqtt_manager_publish(mqtt_alarm_topic_status, "ARMED", strlen("ARMED"), 1, 1);
            ESP_LOGI(TAG, "Risultato pubblicazione: %s", (pub_result == ESP_OK) ? "OK" : "ERRORE");
            
        } else if (strncmp(message, "DISARMED", 8) == 0) {
            
            // Pubblica lo stato aggiornato usando la variabile globale
            ESP_LOGI(TAG, "Pubblicando stato DISARMED su topic %s", mqtt_alarm_topic_status);
            esp_err_t pub_result = mqtt_manager_publish(mqtt_alarm_topic_status, "DISARMED", strlen("DISARMED"), 1, 1);
            ESP_LOGI(TAG, "Risultato pubblicazione: %s", (pub_result == ESP_OK) ? "OK" : "ERRORE");
            
        } else {
            ESP_LOGW(TAG, "Comando ricevuto non riconosciuto: [%s]", message);
        }
    }


}





static void mqtt_state_callback(mqtt_manager_state_t state) {
    switch (state) {
        case MQTT_STATE_CONNECTED:

            ESP_LOGI(TAG, "MQTT connected, subscribing to alarm topic");


            mqtt_manager_send_discovery(&g_config);


            // Riproduce traccia 4 solo al boot
            if (first_boot && dfplayer_queue != NULL) {
                dfplayer_command_t cmd = {.track_number = 4};
                xQueueSend(dfplayer_queue, &cmd, portMAX_DELAY);
                ESP_LOGI(TAG, "Play Traccia 4");
            }
            // Disabilita la riproduzione per le riconnessioni future
            first_boot = false;
            vTaskDelay(pdMS_TO_TICKS(5000)); 


            // Usa le variabili globali già formattate
            mqtt_manager_subscribe(mqtt_alarm_topic_status, 0);
            ESP_LOGI(TAG, "Sottoscritto a %s", mqtt_alarm_topic_status);

            mqtt_manager_subscribe(mqtt_alarm_topic_set, 0);
            ESP_LOGI(TAG, "Sottoscritto a %s", mqtt_alarm_topic_set);

            // Richiedi lo stato corrente dopo la connessione   
            mqtt_manager_publish(mqtt_alarm_topic_get, "", 0, 1, 0);
            ESP_LOGI(TAG, "Richiedendo lo stato attuale dell'allarme");

            break;

        default:
            break;
    }
}





// Callback quando viene rilevata una carta RFID
static void rfid_card_callback(const rfid_uid_t *uid) {
    char uid_str[32];

    rfid_uid_to_string(uid, uid_str, sizeof(uid_str));

    ESP_LOGI(TAG, "Card detected: %s", uid_str);


     // Fai lampeggiare il LED di blu quando viene rilevata una carta
     led_command_t led_cmd = {.red = 0, .green = 0, .blue = 255};
     if (led_queue != NULL) {

         xQueueSend(led_queue, &led_cmd, portMAX_DELAY);
         
         // Opzionale: dopo un breve ritardo, ripristina il colore precedente
         // Nota: questo blocca il thread per 500ms, quindi usalo solo se necessario
         vTaskDelay(pdMS_TO_TICKS(500));
         
         // Ripristina il LED al colore predefinito (verde per disarmato)
         led_cmd.red = 0;
         led_cmd.green = 255;
         led_cmd.blue = 0;

         xQueueSend(led_queue, &last_led_state, portMAX_DELAY);
     }
 

    // Pubblica l'UID su MQTT se siamo connessi
    if (mqtt_manager_is_connected()) {     
        send_rfid_uid(uid_str);
    }

}




// Callback quando viene rimossa una carta RFID
static void rfid_removal_callback(void) {
    ESP_LOGI(TAG, "Card removed");
}


// Callback per quando viene ricevuta una nuova configurazione
void config_received_callback(const app_config_t *new_config) {
    ESP_LOGI(TAG, "New configuration received");
    
    // Salva la configurazione
    memcpy(&g_config, new_config, sizeof(app_config_t));
    config_save(&g_config);
    
    // Imposta il flag per il riavvio
    should_restart = true;
}




void dfplayer_task(void *pvParameter) {
    dfplayer_command_t cmd;
    ESP_LOGI(TAG, "DFPlayer task avviato");
    while (1) {
        //ESP_LOGI(TAG, "DFPlayer: In attesa di comandi dalla coda...");     
        if (xQueueReceive(dfplayer_queue, &cmd, pdMS_TO_TICKS(30000)) == pdTRUE) {
            //ESP_LOGI(TAG, "DFPlayer: Comando ricevuto dalla coda - traccia %d", cmd.track_number);     
            // Riproduce la traccia specificata
            ESP_ERROR_CHECK(dfplayer_play(player, cmd.track_number));   
            // Attendi un momento per permettere al DFPlayer di processare il comando
            vTaskDelay(pdMS_TO_TICKS(500));      
            ESP_LOGI(TAG, "DFPlayer: Comando completato");

        } else {
            // Timeout della coda, fai un controllo periodico
            ESP_LOGV(TAG, "DFPlayer: Timeout coda, pausa");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}






// Funzione per inizializzare l'intero sistema DFPlayer (coda, hardware, task)
void initialize_dfplayer() {
    ESP_LOGI(TAG, "Initializing DFPlayer Mini");
    
    // Inizializza la coda per il DFPlayer
    dfplayer_queue = xQueueCreate(5, sizeof(dfplayer_command_t));
    if (dfplayer_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create DFPlayer queue");
        return;
    }
    
    // Crea il task DFPlayer
    xTaskCreate(dfplayer_task, "DFPLAYER", 4096, NULL, 10, NULL);
    
    // Configure UART for DFPlayer
    uart_config_t uart_config = {
        .baud_rate = CONFIG_DFPLAYER_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_DFPLAYER_UART_PORT, CONFIG_DFPLAYER_UART_BUF_SIZE * 2, CONFIG_DFPLAYER_UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_DFPLAYER_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_DFPLAYER_UART_PORT, CONFIG_DFPLAYER_UART_TXD , CONFIG_DFPLAYER_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialize DFPlayer
    dfplayer_config_t dfplayer_config = {
        .uart_port = CONFIG_DFPLAYER_UART_PORT,
        .debug = false,
        .timeout_ms = 1000,
    };
    
    // Assegna alla variabile globale player
    player = dfplayer_init(&dfplayer_config);
    if (player == NULL) {
        ESP_LOGE(TAG, "Failed to initialize DFPlayer");
        return;
    }
    
    // Wait for DFPlayer to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Set volume to max
    ESP_LOGI(TAG, "Setting volume to max");
    ESP_ERROR_CHECK(dfplayer_volume(player, 30));
    
    // Get current volume
    int16_t volume;
    if (dfplayer_get_volume(player, &volume) == ESP_OK) {
        ESP_LOGI(TAG, "Current volume: %d", volume);
    }
}




/*
static void IRAM_ATTR vl53_isr_handler(void* arg) {
    static uint32_t interrupt_count = 0;
    interrupt_count++;
    uint32_t event_data = interrupt_count;
    xQueueSendFromISR(vl53_queue, &event_data, NULL);
}
*/




/*
static void vl53_service(void* arg) {
    uint32_t event_data = 0;
    VL53L1X_ERROR get_status;
    int64_t update_measurement_time;
    uint8_t RangeStatus = VL53L1_RANGESTATUS_NONE;
    uint16_t Distance = -1;
    bool error;
    int interrupt_count = 0;
    bool object_detected = false;
    uint32_t last_notification_time = 0;
    //uint32_t last_log_time = 0;  // Per limitare anche i log degli interrupt

    ESP_LOGI(TAG, "Task vl53_service avviato, in attesa di interrupt");

    while (1) {

        if (xQueueReceive(vl53_queue, &event_data, pdMS_TO_TICKS(5000))) {

            interrupt_count++;

            uint32_t current_time = esp_timer_get_time() / 1000;  // Converti in millisecondi
            
            // Marca il tempo di inizio
            update_measurement_time = esp_timer_get_time();
            measurement_cycle = update_measurement_time - last_measurement;
            last_measurement = update_measurement_time;
            
            // Ottieni la misurazione e resetta l'interrupt
            get_status = VL53L1X_GetContinuousMeasurement(TOF, &RangeStatus, &Distance);
            
            // Determina se c'è stato un errore di misurazione
            error = get_status != VL53L1_ERROR_NONE;
            error = error || (RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID && RangeStatus != VL53L1_RANGESTATUS_WRAP_TARGET_FAIL);
            
            if (error) {

                range_mm = -1;
                object_detected = false;

            } else {

                range_mm = Distance;  

                // L'interrupt è scattato, quindi c'è un oggetto entro la soglia
                // Se non era già rilevato un oggetto o è passato abbastanza tempo dall'ultima notifica
                
                ESP_LOGI(TAG, "ALLARME1: Oggetto rilevato a %d mm", range_mm);

                if (!object_detected || (current_time - last_notification_time > 4000)) {

                    ESP_LOGI(TAG, "ALLARME: Oggetto rilevato a %d mm", range_mm);
                    object_detected = true;
                    last_notification_time = current_time;

                    if (armed){
                        dfplayer_command_t cmd = {.track_number = 6};
                        xQueueSend(dfplayer_queue, &cmd, portMAX_DELAY);
                        ESP_LOGI(TAG, "Play  traccia 6");
                    } else{
                        dfplayer_command_t cmd = {.track_number = 7};
                        xQueueSend(dfplayer_queue, &cmd, portMAX_DELAY);
                        ESP_LOGI(TAG, "Play  traccia 7");
                    }
                    

                } else {

                    ESP_LOGD(TAG, "Rilevamento troppo frequente, riproduzione saltata (%" PRIu64 " ms dall'ultima)", (uint64_t)(current_time - last_notification_time));

                }

            }

        } else {
            // Timeout: nessun interrupt ricevuto
            object_detected = false;  // Reset del flag dopo timeout
        }

    }
}
*/




/*
void setup_gpio_VL531_interrupt(gpio_num_t gpio_pin, gpio_isr_t isr_handler) {
   
    vl53_queue = xQueueCreate(10, sizeof(uint32_t)); 
    xTaskCreate(vl53_service, "vl53_service", 8192, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    gpio_isr_handler_add(gpio_pin, isr_handler, (void*)&just_a_number);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // Cambiato per rilevare entrambe le transizioni
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}
*/



/*
void reset_vl53l1x() {
    gpio_set_direction(CONFIG_VL53L1_XSHUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_VL53L1_XSHUT_PIN, 0);  // Metti il sensore in reset
    vTaskDelay(pdMS_TO_TICKS(10));         // Aspetta 10ms
    gpio_set_level(CONFIG_VL53L1_XSHUT_PIN, 1);  // Riporta HIGH per attivarlo
    vTaskDelay(pdMS_TO_TICKS(10));         // Attendi stabilizzazione
}
*/




/*
void inizializeVL531() {

    int boot_timeout = 0;
    uint8_t sensorState = 0;
    //VL53L1X_ERROR status = 0;
    
    ESP_LOGI(TAG, "Device Starting...");


    // Inizializzazione I2C
    i2c_init();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    i2c_scan();


    // Attendi il boot del sensore
    while (sensorState == 0 && boot_timeout < 50) { // ~1 secondo di timeout
        VL53L1X_ERROR status = VL53L1X_BootState(TOF, &sensorState);
        if (status != VL53L1_ERROR_NONE) {
            ESP_LOGE(TAG, "Errore durante il polling dello stato del sensore: %d", status);
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Attendi un po' più a lungo prima di riprovare
            continue;  // Riprova comunque, entro il limite del timeout
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
        boot_timeout++;
    }
    
    if (sensorState == 0) {
        ESP_LOGE(TAG, "Timeout nell'attesa del boot del sensore");
        return;
    }


    // Initialize the sensor
    if (VL53L1X_SensorInit(TOF) != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore durante l'inizializzazione del sensore!");
        return;
    }

    
    // Configura la modalità distanza solo se il sensore è pronto
    if (VL53L1X_SetDistanceMode(TOF, 1) != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore setting SetDistanceMode!");
        return;
    }


     // Configura il timing budget
    VL53L1X_SetTimingBudgetInMs(TOF, CONFIG_VL53L1_MEASUREMENT_CYCLE_MS);
    VL53L1X_SetInterMeasurementInMs(TOF, 5 + CONFIG_VL53L1_MEASUREMENT_CYCLE_MS);


    // Configura il GPIO per l'interrupt
    setup_gpio_VL531_interrupt(CONFIG_VL53L1_I2C_DEFAULT_INT, vl53_isr_handler);


    // NUOVA AGGIUNTA: Imposta modalità di misurazione continua
    VL53L1X_SetRangingMode(TOF, 2);  // 2 = RANGING_MODE_CONTINUOUS
    
    
     // Configura il registro GPIO_CONFIG per attivare l'interrupt sulla soglia
    VL53L1_WrByte(TOF, SYSTEM__INTERRUPT_CONFIG_GPIO, 0x04); // 0x04 = interrupt on threshold event
    ESP_LOGI(TAG, "Interrupt configurato per evento di soglia");


    // Configura soglia interrupt: attiva quando la distanza è tra 0 e 500mm
    if (VL53L1X_SetDistanceThreshold(TOF, 0, 500 , 3, 0) != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore nella configurazione della soglia di distanza!");
        return;
    }
    ESP_LOGI(TAG, "Soglia di interrupt configurata: attiva quando 0 < distanza < 500mm");

    vTaskDelay(1000 / portTICK_PERIOD_MS);


    // Avvia le misurazioni continue
    ESP_LOGI(TAG, "VL53L1X Ultra Lite Driver running...");
    if (VL53L1X_StartRanging(TOF) != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore nell'avvio delle misurazioni!");
        return;
    }
    ESP_LOGI(TAG, "Misurazione continua avviata");

    
}
*/




void on_uri_registered(const char *uri, httpd_method_t method) {
    char *method_str = "";

    switch (method) {
        case HTTP_GET: method_str = "GET"; 

        dfplayer_command_t cmd = {.track_number = 5};
        xQueueSend(dfplayer_queue, &cmd, portMAX_DELAY);
        ESP_LOGI(TAG, "Play Traccia 5");

        break;

        case HTTP_POST: method_str = "POST"; 
        break;

        case HTTP_PUT: method_str = "PUT"; 
        break;

        case HTTP_DELETE: method_str = "DELETE"; 
        break;

        default: method_str = "UNKNOWN"; 
        break;
    }

    ESP_LOGI(TAG, "Webserver registered URI: %s, method: %s", uri, method_str);



}








void app_main(void) {

    ESP_LOGI(TAG, "Starting application");

     // Inizializza il DFPlayer
     initialize_dfplayer();


    // Prima di chiamare webserver_start:
    webserver_set_uri_registered_callback(on_uri_registered);


    // Inizializza NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);



    // Configura il pin del pulsante di configurazione
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_CONFIG_PIN_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);


    // Carica la configurazione
    if (config_load(&g_config) != ESP_OK) {
        config_reset();
    }


    // Inizializza i Topic MQTT usando i valori caricati dalla configurazione
    init_mqtt_topics();



    // Controlla se entrare in modalità configurazione
    if (gpio_get_level(CONFIG_CONFIG_PIN_MODE) == 0 || !config_is_valid()) {

        ESP_LOGI(TAG, "--------> Entering configuration mode");

        app_state = APP_STATE_CONFIG_MODE;

        // Inizializza WiFi in modalità AP
        wifi_manager_init(wifi_state_callback);

        wifi_manager_start_ap();

        
        // Inizializza e avvia il captive portal
        captive_portal_init(webserver_get_server_handle()); // Avrai bisogno di esporre l'handle del server nel tuo webserver.c
        captive_portal_start();


    } else {

        ESP_LOGI(TAG, "Starting in normal mode");

        app_state = APP_STATE_NORMAL_MODE;


        // Inizilizzo sensore TOF
        //inizializeVL531();


        // Inizializza tutti i moduli
        wifi_manager_init(wifi_state_callback);

         // Avvia WiFi in modalità station
        wifi_manager_start_sta(&g_config);

        mqtt_manager_init(&g_config, mqtt_message_callback, mqtt_state_callback);


        if (g_config.mqtt_enable_availability) {
            start_availability_task(g_config.device_id);
        }


        start_availability_task(g_config.device_id);     


        rfid_manager_init(rfid_card_callback, rfid_removal_callback);


        // Avvia il lettore RFID
        rfid_manager_start();


        // Inizializza il LED
        led_init(); 


        // Crea la coda e il task LED
        led_queue = xQueueCreate(5, sizeof(led_command_t));
        if (led_queue == NULL) {
            ESP_LOGE(TAG, "Errore creazione coda LED");
            return;
        }
        xTaskCreate(led_control_task, "LED_CTRL", 4096, NULL, 10, NULL);




    }

    



    // Main loop
    while (1) {

        if (should_restart) {
            ESP_LOGI(TAG, "Restarting system...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Previene watchdog timeout
    }


}