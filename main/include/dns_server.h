/**
 * @file dns_server.h
 * @brief DNS server semplice per captive portal
 */

 #ifndef DNS_SERVER_H
 #define DNS_SERVER_H
 
 #include "esp_err.h"
 
 /**
  * @brief Avvia il server DNS per il captive portal
  * 
  * @return ESP_OK se l'avvio è avvenuto con successo, altrimenti un codice di errore
  */
 esp_err_t dns_server_start(void);
 
 /**
  * @brief Ferma il server DNS
  * 
  * @return ESP_OK se l'arresto è avvenuto con successo, altrimenti un codice di errore
  */
 esp_err_t dns_server_stop(void);
 
 #endif /* DNS_SERVER_H */