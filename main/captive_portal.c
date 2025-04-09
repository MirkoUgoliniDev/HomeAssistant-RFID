/**
 * @file captive_portal.c
 * @brief Implementazione del captive portal per ESP32-S2
 */

 #include "captive_portal.h"
 #include "dns_server.h"
 #include "esp_log.h"
 #include "esp_wifi.h"
 #include "esp_netif.h"
 #include <string.h>
 
 static const char *TAG = "captive_portal";
 static httpd_handle_t web_server = NULL;
 
 /**
  * @brief Ottiene l'URL di redirect usando l'indirizzo IP attuale dell'AP
  * @param url Buffer dove salvare l'URL generato
  * @param url_size Dimensione del buffer
  */
 static void get_redirect_url(char *url, size_t url_size) {
     // Verifica validità parametri
     if (url == NULL || url_size < 16) { // 16 bytes: spazio minimo per "http://x.x.x.x/"
         ESP_LOGE(TAG, "Buffer URL non valido o troppo piccolo");
         return;
     }
     
     esp_netif_ip_info_t ip_info;
     
     // Ottieni l'interfaccia AP
     esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
     if (netif == NULL) {
         // Se non riesci a ottenere l'interfaccia, usa il valore predefinito
         snprintf(url, url_size, "http://192.168.4.1/");
         ESP_LOGW(TAG, "Impossibile ottenere l'interfaccia AP, usando indirizzo predefinito");
         return;
     }
     
     // Ottieni le informazioni sull'IP dall'interfaccia
     if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
         // Se non riesci a ottenere l'IP, usa il valore predefinito
         snprintf(url, url_size, "http://192.168.4.1/");
         ESP_LOGW(TAG, "Impossibile ottenere l'IP dell'AP, usando indirizzo predefinito");
         return;
     }
     
     // Formatta l'URL di reindirizzamento usando l'IP attuale
     snprintf(url, url_size, "http://"IPSTR"/", IP2STR(&ip_info.ip));
     ESP_LOGI(TAG, "URL di reindirizzamento: %s", url);
 }
 
 /**
  * @brief Handler per il captive portal di Android
  * Reindirizza /generate_204 richiesto da Android alla pagina di configurazione
  */
 static esp_err_t captive_portal_android_handler(httpd_req_t *req) {
     char redirect_url[64];
     get_redirect_url(redirect_url, sizeof(redirect_url));
     
     ESP_LOGI(TAG, "Captive Portal Android: reindirizzamento alla pagina di configurazione");
     httpd_resp_set_status(req, "302 Found");
     httpd_resp_set_hdr(req, "Location", redirect_url);
     httpd_resp_send(req, NULL, 0);
     return ESP_OK;
 }
 
 /**
  * @brief Handler per il captive portal di Windows/Microsoft
  * Reindirizza /connecttest.txt richiesto da Windows alla pagina di configurazione
  */
 static esp_err_t captive_portal_windows_handler(httpd_req_t *req) {
     char redirect_url[64];
     get_redirect_url(redirect_url, sizeof(redirect_url));
     
     ESP_LOGI(TAG, "Captive Portal Windows: reindirizzamento alla pagina di configurazione");
     httpd_resp_set_status(req, "302 Found");
     httpd_resp_set_hdr(req, "Location", redirect_url);
     httpd_resp_send(req, NULL, 0);
     return ESP_OK;
 }
 
 /**
  * @brief Handler per il captive portal di iOS/Apple
  * Reindirizza /hotspot-detect.html richiesto da iOS alla pagina di configurazione
  */
 static esp_err_t captive_portal_apple_handler(httpd_req_t *req) {
     char redirect_url[64];
     get_redirect_url(redirect_url, sizeof(redirect_url));
     
     ESP_LOGI(TAG, "Captive Portal Apple: reindirizzamento alla pagina di configurazione");
     httpd_resp_set_status(req, "302 Found");
     httpd_resp_set_hdr(req, "Location", redirect_url);
     httpd_resp_send(req, NULL, 0);
     return ESP_OK;
 }
 
 /**
  * @brief Handler generico per tutte le altre richieste del captive portal
  * Reindirizza qualsiasi altra richiesta alla pagina di configurazione
  */
 static esp_err_t captive_portal_generic_handler(httpd_req_t *req) {
     char redirect_url[64];
     get_redirect_url(redirect_url, sizeof(redirect_url));
     
     ESP_LOGI(TAG, "Captive Portal Generico (%s): reindirizzamento alla pagina di configurazione", req->uri);
     httpd_resp_set_status(req, "302 Found");
     httpd_resp_set_hdr(req, "Location", redirect_url);
     httpd_resp_send(req, NULL, 0);
     return ESP_OK;
 }
 
 esp_err_t captive_portal_init(httpd_handle_t server) {
     if (server == NULL) {
         ESP_LOGE(TAG, "Handle del server HTTP non valido");
         return ESP_ERR_INVALID_ARG;
     }
     
     web_server = server;
     
     // Registra handler per Android (generate_204)
     httpd_uri_t android_portal = {
         .uri = "/generate_204",
         .method = HTTP_GET,
         .handler = captive_portal_android_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &android_portal) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler Android");
         return ESP_FAIL;
     }
     
     // Handler alternativo per Android (gen_204)
     httpd_uri_t android_portal_alt = {
         .uri = "/gen_204",
         .method = HTTP_GET,
         .handler = captive_portal_android_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &android_portal_alt) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler Android alternativo");
         return ESP_FAIL;
     }
     
     // Handler per Windows (connecttest.txt)
     httpd_uri_t windows_portal = {
         .uri = "/connecttest.txt",
         .method = HTTP_GET,
         .handler = captive_portal_windows_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &windows_portal) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler Windows");
         return ESP_FAIL;
     }
     
     // Handler per Apple/iOS (hotspot-detect.html)
     httpd_uri_t apple_portal = {
         .uri = "/hotspot-detect.html",
         .method = HTTP_GET,
         .handler = captive_portal_apple_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &apple_portal) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler Apple");
         return ESP_FAIL;
     }
     
     // Handler per Apple/iOS alternativo (captive.apple.com)
     httpd_uri_t apple_portal_alt = {
         .uri = "/library/test/success.html",
         .method = HTTP_GET,
         .handler = captive_portal_apple_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &apple_portal_alt) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler Apple alternativo");
         return ESP_FAIL;
     }
     
     // Handler generico per tutte le altre richieste (wildcard)
     // Questo deve essere registrato per ultimo perché è un catch-all
     httpd_uri_t generic_portal = {
         .uri = "/*",
         .method = HTTP_GET,
         .handler = captive_portal_generic_handler,
         .user_ctx = NULL
     };
     
     if (httpd_register_uri_handler(web_server, &generic_portal) != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile registrare handler generico");
         return ESP_FAIL;
     }
     
     ESP_LOGI(TAG, "Captive portal inizializzato con successo");
     return ESP_OK;
 }
 
 esp_err_t captive_portal_start(void) {
     ESP_LOGI(TAG, "Avvio captive portal");
     
     // Avvia il server DNS
     esp_err_t err = dns_server_start();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile avviare il server DNS: %s", esp_err_to_name(err));
         return err;
     }
     
     ESP_LOGI(TAG, "Captive portal avviato con successo");
     return ESP_OK;
 }
 
 esp_err_t captive_portal_stop(void) {
     ESP_LOGI(TAG, "Arresto captive portal");
     
     // Arresta il server DNS
     esp_err_t err = dns_server_stop();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Impossibile arrestare il server DNS: %s", esp_err_to_name(err));
         // Continua comunque l'arresto
     }
     
     // Reset della variabile del server web
     web_server = NULL;
     
     ESP_LOGI(TAG, "Captive portal arrestato con successo");
     return ESP_OK;
 }