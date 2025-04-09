#ifndef _RFID_MANAGER_H_
#define _RFID_MANAGER_H_

#include <esp_err.h>
#include "config.h"

// Struttura per l'UID della carta
typedef struct {
    uint8_t bytes[10];    // Buffer per l'UID
    uint8_t length;       // Lunghezza effettiva dell'UID
} rfid_uid_t;

// Callback per quando viene rilevata una carta
typedef void (*rfid_card_callback_t)(const rfid_uid_t *uid);

// Callback per quando una carta viene rimossa
typedef void (*rfid_removal_callback_t)(void);

/**
 * @brief Inizializza il manager RFID
 * @param card_cb Callback per la rilevazione carta
 * @param removal_cb Callback per la rimozione carta
 * @return ESP_OK se successo
 */
esp_err_t rfid_manager_init(rfid_card_callback_t card_cb, 
                           rfid_removal_callback_t removal_cb);

/**
 * @brief Avvia il lettore RFID
 * @return ESP_OK se successo
 */
esp_err_t rfid_manager_start(void);

/**
 * @brief Ferma il lettore RFID
 * @return ESP_OK se successo
 */
esp_err_t rfid_manager_stop(void);

/**
 * @brief Converte un UID in stringa esadecimale
 * @param uid UID da convertire
 * @param str Buffer dove salvare la stringa
 * @param str_size Dimensione del buffer
 * @return ESP_OK se successo
 */
esp_err_t rfid_uid_to_string(const rfid_uid_t *uid, char *str, size_t str_size);

/**
 * @brief Verifica se una carta è presente
 * @return true se una carta è presente
 */
bool rfid_is_card_present(void);

#endif // _RFID_MANAGER_H_