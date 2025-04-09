/**
 * @file captive_portal.h
 * @brief Gestione del captive portal per ESP32-S2
 */

 #ifndef CAPTIVE_PORTAL_H
 #define CAPTIVE_PORTAL_H
 
 #include "esp_err.h"
 #include "esp_http_server.h"
 
 /**
  * @brief Inizializza il captive portal
  * 
  * @param server Handle del server HTTP esistente
  * @return ESP_OK se l'inizializzazione è avvenuta con successo, altrimenti un codice di errore
  */
 esp_err_t captive_portal_init(httpd_handle_t server);
 
 /**
  * @brief Avvia tutti i componenti del captive portal (server DNS, ecc.)
  * 
  * @return ESP_OK se l'avvio è avvenuto con successo, altrimenti un codice di errore
  */
 esp_err_t captive_portal_start(void);
 
 /**
  * @brief Arresta tutti i componenti del captive portal
  * 
  * @return ESP_OK se l'arresto è avvenuto con successo, altrimenti un codice di errore
  */
 esp_err_t captive_portal_stop(void);
 
 #endif /* CAPTIVE_PORTAL_H */