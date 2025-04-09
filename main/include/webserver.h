/**
 * @file webserver.h
 * @brief Header file for the ESP32 configuration web server
 */

 #ifndef _WEBSERVER_H_
 #define _WEBSERVER_H_
 
 #include <esp_err.h>
 #include "config.h"
 #include "esp_http_server.h"
 

/**
 * @brief Restituisce l'handle del server HTTP attivo
 * 
 * @return httpd_handle_t Handle del server se attivo, NULL altrimenti
 */
httpd_handle_t webserver_get_server_handle(void);




 /**
  * @typedef webserver_config_callback_t
  * @brief Callback function type for receiving a new configuration
  * @param new_config Pointer to the new configuration
  */
 typedef void (*webserver_config_callback_t)(const app_config_t *new_config);
 


 /**
  * @brief Start the web server
  * @param config_callback Callback function to receive new configurations
  * @param config Pointer to current configuration (can be NULL)
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t webserver_start(webserver_config_callback_t config_callback, app_config_t *config);
 


 // Aggiungi questa definizione di tipo nel file webserver.h
typedef void (*webserver_uri_registered_callback_t)(const char *uri, httpd_method_t method);

// Aggiungi questa dichiarazione di funzione
void webserver_set_uri_registered_callback(webserver_uri_registered_callback_t callback);


 /**
  * @brief Stop the web server
  * @return ESP_OK if successful, otherwise an error code
  */
 esp_err_t webserver_stop(void);
 



 /**
  * @brief Check if the web server is running
  * @return true if the server is running, false otherwise
  */
 bool webserver_is_running(void);
 



 /**
  * @brief Get the URL of the web server
  * @return URL string or NULL if server is not running
  */
 const char* webserver_get_url(void);
 






 #endif /* _WEBSERVER_H_ */