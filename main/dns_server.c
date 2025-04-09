/**
 * @file dns_server.c
 * @brief Implementazione del server DNS semplice per captive portal
 */

 #include "dns_server.h"
 #include "esp_log.h"
 #include "lwip/err.h"
 #include "lwip/sockets.h"
 #include "lwip/sys.h"
 #include "lwip/netdb.h"
 #include "lwip/dns.h"
 #include "esp_netif.h"
 #include "esp_wifi.h"
 #include "sdkconfig.h"
 
 static const char *TAG = "dns_server";
 
 // Handle del task del server DNS
 static TaskHandle_t dns_server_task_handle = NULL;
 
 // Socket del server DNS
 static int sock = -1;
 
 // Struttura del pacchetto DNS
 typedef struct __attribute__((packed)) {
     uint16_t id;
     uint16_t flags;
     uint16_t qdcount;
     uint16_t ancount;
     uint16_t nscount;
     uint16_t arcount;
 } dns_header_t;
 
 // Struttura della query DNS
 typedef struct __attribute__((packed)) {
     uint16_t type;
     uint16_t class;
 } dns_query_t;
 
 // Struttura della risposta DNS
 typedef struct __attribute__((packed)) {
     uint16_t name;
     uint16_t type;
     uint16_t class;
     uint32_t ttl;
     uint16_t rdlength;
     uint32_t rd_data;
 } dns_answer_t;
 
 /**
  * @brief Task del server DNS
  * 
  * @param pvParameters Parametri del task
  */
 static void dns_server_task(void *pvParameters) {
     uint8_t rx_buffer[512];
     uint8_t tx_buffer[512];
     struct sockaddr_in server_addr, remote_addr;
     socklen_t socklen = sizeof(remote_addr);
     int len;
     esp_netif_ip_info_t ip_info;
     
     // Creazione del socket UDP
     sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
     if (sock < 0) {
         ESP_LOGE(TAG, "Impossibile creare il socket: %d", sock);
         vTaskDelete(NULL);
         return;
     }
     
     // Binding del socket alla porta 53 (DNS)
     server_addr.sin_family = AF_INET;
     server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
     server_addr.sin_port = htons(53);
     if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
         ESP_LOGE(TAG, "Impossibile eseguire il binding del socket");
         close(sock);
         sock = -1;
         vTaskDelete(NULL);
         return;
     }
     
     // Ottenimento dell'indirizzo IP dell'AP
     esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
     
     ESP_LOGI(TAG, "Server DNS in esecuzione sulla porta 53");
     
     // Loop principale
     while (1) {
         // Ricezione della query DNS
         len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&remote_addr, &socklen);
         if (len < sizeof(dns_header_t)) {
             continue;  // Pacchetto non valido
         }
         
         // Analisi del pacchetto DNS
         dns_header_t *query_header = (dns_header_t *)rx_buffer;
         dns_header_t *reply_header = (dns_header_t *)tx_buffer;
         
         // Verifica se è una query standard
         if (ntohs(query_header->flags) & 0x8000) {
             continue;  // Non è una query
         }
         
         // Copia della query nella risposta
         memcpy(tx_buffer, rx_buffer, len);
         
         // Impostazione dei flag di risposta
         reply_header->flags = htons(0x8180);  // Risposta standard, nessun errore
         reply_header->ancount = htons(1);     // Una risposta
         
         // Ricerca della fine del nome della query
         int name_end = sizeof(dns_header_t);
         while (name_end < len && rx_buffer[name_end] != 0) {
             name_end += rx_buffer[name_end] + 1;
         }
         name_end++;  // Salta il byte 0
         
         if (name_end + sizeof(dns_query_t) > len) {
             continue;  // Query non valida
         }
         
         // Ottenimento del tipo di query
         dns_query_t *query = (dns_query_t *)(rx_buffer + name_end);
         
         // Rispondi solo alle query di tipo A (indirizzo IPv4)
         if (ntohs(query->type) != 1) {
             continue;
         }
         
         // Aggiunta della risposta
         int answer_pos = len;
         dns_answer_t *answer = (dns_answer_t *)(tx_buffer + answer_pos);
         answer->name = htons(0xC00C);    // Puntatore al nome nella query
         answer->type = htons(1);         // Record A
         answer->class = htons(1);        // Classe IN
         answer->ttl = htonl(60);         // TTL 60 secondi
         answer->rdlength = htons(4);     // L'indirizzo IPv4 è di 4 byte
         answer->rd_data = ip_info.ip.addr;  // Restituisci l'IP dell'AP per tutti i domini
         
         // Invio della risposta
         sendto(sock, tx_buffer, answer_pos + sizeof(dns_answer_t), 0, 
                (struct sockaddr *)&remote_addr, socklen);
     }
     
     // Mai raggiunto
     close(sock);
     sock = -1;
     vTaskDelete(NULL);
 }
 
 esp_err_t dns_server_start(void) {
     if (dns_server_task_handle != NULL) {
         ESP_LOGW(TAG, "Server DNS già in esecuzione");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Creazione del task del server DNS
     BaseType_t ret = xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &dns_server_task_handle);
     if (ret != pdPASS) {
         ESP_LOGE(TAG, "Impossibile creare il task del server DNS: %d", ret);
         return ESP_ERR_NO_MEM;
     }
     
     return ESP_OK;
 }
 
 esp_err_t dns_server_stop(void) {
     if (dns_server_task_handle == NULL) {
         ESP_LOGW(TAG, "Server DNS non in esecuzione");
         return ESP_ERR_INVALID_STATE;
     }
     
     // Chiusura del socket per far uscire il task
     if (sock >= 0) {
         close(sock);
         sock = -1;
     }
     
     // Eliminazione del task
     vTaskDelete(dns_server_task_handle);
     dns_server_task_handle = NULL;
     
     return ESP_OK;
 }