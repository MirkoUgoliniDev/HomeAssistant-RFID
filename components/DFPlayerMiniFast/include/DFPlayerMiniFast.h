/**
 * @file dfplayer_mini.h
 * @brief ESP-IDF driver for DFPlayerMini MP3 player module
 *
 * This is an ESP-IDF port of the DFPlayerMini_Fast Arduino library
 * for the YX5200-24SS MP3 player module.
 * 
 * Original Arduino library by Power_Broker
 * ESP-IDF port created at user's request
 */

#ifndef DFPLAYER_MINI_H
#define DFPLAYER_MINI_H

#include "esp_err.h"
#include "driver/uart.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief DFPlayer namespace for constants
 */
typedef enum {
    /** Packet Values */
    DFP_STACK_SIZE      = 10,   // total number of bytes in a stack/packet (same for cmds and queries)
    DFP_SB              = 0x7E, // start byte
    DFP_VER             = 0xFF, // version
    DFP_LEN             = 0x6,  // number of bytes after "LEN" (except for checksum data and EB)
    DFP_FEEDBACK        = 1,    // feedback requested
    DFP_NO_FEEDBACK     = 0,    // no feedback requested
    DFP_EB              = 0xEF, // end byte

    /** Control Command Values */
    DFP_NEXT            = 0x01,
    DFP_PREV            = 0x02,
    DFP_PLAY            = 0x03,
    DFP_INC_VOL         = 0x04,
    DFP_DEC_VOL         = 0x05,
    DFP_VOLUME          = 0x06,
    DFP_EQ              = 0x07,
    DFP_PLAYBACK_MODE   = 0x08,
    DFP_PLAYBACK_SRC    = 0x09,
    DFP_STANDBY         = 0x0A,
    DFP_NORMAL          = 0x0B,
    DFP_RESET           = 0x0C,
    DFP_PLAYBACK        = 0x0D,
    DFP_PAUSE           = 0x0E,
    DFP_SPEC_FOLDER     = 0x0F,
    DFP_VOL_ADJ         = 0x10,
    DFP_REPEAT_PLAY     = 0x11,
    DFP_USE_MP3_FOLDER  = 0x12,
    DFP_INSERT_ADVERT   = 0x13,
    DFP_SPEC_TRACK_3000 = 0x14,
    DFP_STOP_ADVERT     = 0x15,
    DFP_STOP            = 0x16,
    DFP_REPEAT_FOLDER   = 0x17,
    DFP_RANDOM_ALL      = 0x18,
    DFP_REPEAT_CURRENT  = 0x19,
    DFP_SET_DAC         = 0x1A,

    /** Query Command Values */
    DFP_SEND_INIT        = 0x3F,
    DFP_RETRANSMIT       = 0x40,
    DFP_REPLY            = 0x41,
    DFP_GET_STATUS       = 0x42,
    DFP_GET_VOL          = 0x43,
    DFP_GET_EQ           = 0x44,
    DFP_GET_MODE         = 0x45,
    DFP_GET_VERSION      = 0x46,
    DFP_GET_TF_FILES     = 0x47,
    DFP_GET_U_FILES      = 0x48,
    DFP_GET_FLASH_FILES  = 0x49,
    DFP_KEEP_ON          = 0x4A,
    DFP_GET_TF_TRACK     = 0x4B,
    DFP_GET_U_TRACK      = 0x4C,
    DFP_GET_FLASH_TRACK  = 0x4D,
    DFP_GET_FOLDER_FILES = 0x4E,
    DFP_GET_FOLDERS      = 0x4F,

    /** EQ Values */
    DFP_EQ_NORMAL       = 0,
    DFP_EQ_POP          = 1,
    DFP_EQ_ROCK         = 2,
    DFP_EQ_JAZZ         = 3,
    DFP_EQ_CLASSIC      = 4,
    DFP_EQ_BASE         = 5,

    /** Mode Values */
    DFP_REPEAT          = 0,
    DFP_FOLDER_REPEAT   = 1,
    DFP_SINGLE_REPEAT   = 2,
    DFP_RANDOM          = 3,

    /** Playback Source Values */
    DFP_U               = 1,
    DFP_TF              = 2,
    DFP_AUX             = 3,
    DFP_SLEEP           = 4,
    DFP_FLASH           = 5,

    /** Base Volume Adjust Value */
    DFP_VOL_ADJUST      = 0x10,

    /** Repeat Play Values */
    DFP_STOP_REPEAT     = 0,
    DFP_START_REPEAT    = 1
} dfplayer_cmd_t;


/**
 * @brief MP3 response packet parsing states
 */
typedef enum {
    DFP_FIND_START_BYTE,
    DFP_FIND_VER_BYTE,
    DFP_FIND_LEN_BYTE,
    DFP_FIND_COMMAND_BYTE,
    DFP_FIND_FEEDBACK_BYTE,
    DFP_FIND_PARAM_MSB,
    DFP_FIND_PARAM_LSB,
    DFP_FIND_CHECKSUM_MSB,
    DFP_FIND_CHECKSUM_LSB,
    DFP_FIND_END_BYTE
} dfplayer_state_t;



/**
 * @brief Struct to store entire serial datapacket used for MP3 config/control
 */
typedef struct {
    uint8_t start_byte;
    uint8_t version;
    uint8_t length;
    uint8_t commandValue;
    uint8_t feedbackValue;
    uint8_t paramMSB;
    uint8_t paramLSB;
    uint8_t checksumMSB;
    uint8_t checksumLSB;
    uint8_t end_byte;
} dfplayer_stack_t;



/**
 * @brief DFPlayer configuration
 */
typedef struct {
    uart_port_t uart_port;    // UART port number
    bool debug;               // Debug output flag
    uint32_t timeout_ms;      // Timeout for responses in ms
} dfplayer_config_t;



/**
 * @brief DFPlayer handle type
 */
typedef struct dfplayer_handle* dfplayer_handle_t;



/**
 * @brief Initialize the DFPlayer
 * 
 * @param config Configuration structure
 * @return dfplayer_handle_t Handle to the DFPlayer instance, or NULL if failed
 */
dfplayer_handle_t dfplayer_init(dfplayer_config_t *config);

/**
 * @brief Free resources used by the DFPlayer
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_deinit(dfplayer_handle_t player);


/**
 * @brief Play the next song in chronological order
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play_next(dfplayer_handle_t player);

/**
 * @brief Play the previous song in chronological order
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play_previous(dfplayer_handle_t player);

/**
 * @brief Play a specific track
 * 
 * @param player Handle to the DFPlayer instance
 * @param track_num The track number to play
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play(dfplayer_handle_t player, uint16_t track_num);


/**
 * @brief Stop the current playback
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_stop(dfplayer_handle_t player);



/**
 * @brief Play a specific track in the folder named "MP3"
 * 
 * @param player Handle to the DFPlayer instance
 * @param track_num The track number to play
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play_from_mp3_folder(dfplayer_handle_t player, uint16_t track_num);

/**
 * @brief Play a specific track in a specific folder
 * 
 * @param player Handle to the DFPlayer instance
 * @param folder_num The folder number
 * @param track_num The track number to play
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play_folder(dfplayer_handle_t player, uint8_t folder_num, uint8_t track_num);

/**
 * @brief Play a specific track from a specific folder with 4-digit numbering support
 * 
 * @param player Handle to the DFPlayer instance
 * @param folder_num The folder number (1-15)
 * @param track_num The track number to play (1-3000)
 * @return ESP_OK on success
 */
esp_err_t dfplayer_play_large_folder(dfplayer_handle_t player, uint8_t folder_num, uint16_t track_num);

/**
 * @brief Set volume level (0-30)
 * 
 * @param player Handle to the DFPlayer instance
 * @param volume Volume level (0-30)
 * @return ESP_OK on success
 */
esp_err_t dfplayer_volume(dfplayer_handle_t player, uint8_t volume);

/**
 * @brief Increment volume by 1
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_volume_up(dfplayer_handle_t player);

/**
 * @brief Decrement volume by 1
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_volume_down(dfplayer_handle_t player);

/**
 * @brief Set EQ mode
 * 
 * @param player Handle to the DFPlayer instance
 * @param eq EQ setting (0-5)
 * @return ESP_OK on success
 */
esp_err_t dfplayer_eq_select(dfplayer_handle_t player, uint8_t eq);

/**
 * @brief Loop a specific track
 * 
 * @param player Handle to the DFPlayer instance
 * @param track_num The track number to loop
 * @return ESP_OK on success
 */
esp_err_t dfplayer_loop(dfplayer_handle_t player, uint16_t track_num);

/**
 * @brief Specify the playback source
 * 
 * @param player Handle to the DFPlayer instance
 * @param source The playback source (1-5)
 * @return ESP_OK on success
 */
esp_err_t dfplayer_playback_source(dfplayer_handle_t player, uint8_t source);

/**
 * @brief Put the MP3 player in standby mode
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_standby(dfplayer_handle_t player);

/**
 * @brief Pull the MP3 player out of standby mode
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_normal_mode(dfplayer_handle_t player);

/**
 * @brief Reset all settings to factory default
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_reset(dfplayer_handle_t player);

/**
 * @brief Resume playing current track
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_resume(dfplayer_handle_t player);

/**
 * @brief Pause playing current track
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_pause(dfplayer_handle_t player);

/**
 * @brief Play all tracks in a random order
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_random_all(dfplayer_handle_t player);

/**
 * @brief Turn on DAC
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_start_dac(dfplayer_handle_t player);

/**
 * @brief Turn off DAC
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_stop_dac(dfplayer_handle_t player);

/**
 * @brief Put the MP3 player into sleep mode
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_sleep(dfplayer_handle_t player);

/**
 * @brief Pull the MP3 player out of sleep mode
 * 
 * @param player Handle to the DFPlayer instance
 * @return ESP_OK on success
 */
esp_err_t dfplayer_wake_up(dfplayer_handle_t player);

/**
 * @brief Determine if a track is currently playing
 * 
 * @param player Handle to the DFPlayer instance
 * @param is_playing Pointer to store result (true if playing)
 * @return ESP_OK on success
 */
esp_err_t dfplayer_is_playing(dfplayer_handle_t player, bool *is_playing);

/**
 * @brief Get current volume level
 * 
 * @param player Handle to the DFPlayer instance
 * @param volume Pointer to store volume value
 * @return ESP_OK on success
 */
esp_err_t dfplayer_get_volume(dfplayer_handle_t player, int16_t *volume);

/**
 * @brief Get current EQ setting
 * 
 * @param player Handle to the DFPlayer instance
 * @param eq Pointer to store EQ value
 * @return ESP_OK on success
 */
esp_err_t dfplayer_get_eq(dfplayer_handle_t player, int16_t *eq);

/**
 * @brief Get current playback mode
 * 
 * @param player Handle to the DFPlayer instance
 * @param mode Pointer to store mode value
 * @return ESP_OK on success
 */
esp_err_t dfplayer_get_mode(dfplayer_handle_t player, int16_t *mode);

/**
 * @brief Get number of tracks on SD card
 * 
 * @param player Handle to the DFPlayer instance
 * @param count Pointer to store track count
 * @return ESP_OK on success
 */
esp_err_t dfplayer_get_sd_track_count(dfplayer_handle_t player, int16_t *count);

/**
 * @brief Get current track on SD card
 * 
 * @param player Handle to the DFPlayer instance
 * @param track Pointer to store current track
 * @return ESP_OK on success
 */
esp_err_t dfplayer_get_current_sd_track(dfplayer_handle_t player, int16_t *track);

#ifdef __cplusplus
}
#endif

#endif /* DFPLAYER_MINI_H */