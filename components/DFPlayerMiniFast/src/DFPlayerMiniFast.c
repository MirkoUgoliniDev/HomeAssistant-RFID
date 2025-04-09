/**
 * @file dfplayer_mini.c
 * @brief ESP-IDF driver for DFPlayerMini MP3 player module
 *
 * This is an ESP-IDF port of the DFPlayerMini_Fast Arduino library
 * for the YX5200-24SS MP3 player module.
 * 
 * Original Arduino library by Power_Broker
 * ESP-IDF port created at user's request
 */

 #include "DFPlayerMiniFast.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include <string.h>
 #include "esp_timer.h"

 
 static const char *TAG = "DFPlayer";
 
 /**
  * @brief Structure to hold the DFPlayer instance data
  */
 struct dfplayer_handle {
     uart_port_t uart_port;
     bool debug;
     uint32_t timeout_ms;
     dfplayer_stack_t send_stack;
     dfplayer_stack_t rec_stack;
     dfplayer_state_t state;
 };
 



 /**
  * @brief Calculate and set checksum for the command packet
  * 
  * @param stack Pointer to the stack structure
  */
 static void dfplayer_find_checksum(dfplayer_stack_t *stack) {
     uint16_t checksum = (~(stack->version + stack->length + stack->commandValue + stack->feedbackValue + stack->paramMSB + stack->paramLSB)) + 1;
     stack->checksumMSB = checksum >> 8;
     stack->checksumLSB = checksum & 0xFF;
 }
 


 /**
  * @brief Send a command packet to the DFPlayer
  * 
  * @param player Handle to the DFPlayer instance
  * @return ESP_OK on success
  */
 static esp_err_t dfplayer_send_data(dfplayer_handle_t player) {
     uint8_t buffer[10];
     
     buffer[0] = player->send_stack.start_byte;
     buffer[1] = player->send_stack.version;
     buffer[2] = player->send_stack.length;
     buffer[3] = player->send_stack.commandValue;
     buffer[4] = player->send_stack.feedbackValue;
     buffer[5] = player->send_stack.paramMSB;
     buffer[6] = player->send_stack.paramLSB;
     buffer[7] = player->send_stack.checksumMSB;
     buffer[8] = player->send_stack.checksumLSB;
     buffer[9] = player->send_stack.end_byte;
 
     int written = uart_write_bytes(player->uart_port, (const char*)buffer, 10);
     
     if (written != 10) {
         ESP_LOGE(TAG, "Failed to send complete packet, only sent %d bytes", written);
         return ESP_FAIL;
     }
 
     if (player->debug) {
         ESP_LOGV(TAG, "Sent packet: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9]);
     }
     
     return ESP_OK;
 }
 



 /**
  * @brief Parse feedback from the DFPlayer
  * 
  * @param player Handle to the DFPlayer instance
  * @return true if successful parsing, false otherwise
  */
 static bool dfplayer_parse_feedback(dfplayer_handle_t player) {
     uint8_t rec_char;
     int64_t start_time = esp_timer_get_time() / 1000; // Convert to ms
     
     while ((esp_timer_get_time() / 1000) - start_time < player->timeout_ms) {
         size_t available;
         
         // Check if data is available
         if (uart_get_buffered_data_len(player->uart_port, &available) == ESP_OK && available > 0) {
             if (uart_read_bytes(player->uart_port, &rec_char, 1, 0) == 1) {

                 if (player->debug) {
                     ESP_LOGV(TAG, "Received byte: 0x%02X, State: %d", rec_char, player->state);
                 }
                 
                 switch (player->state) {
                     case DFP_FIND_START_BYTE:
                         if (rec_char == DFP_SB) {
                             player->rec_stack.start_byte = rec_char;
                             player->state = DFP_FIND_VER_BYTE;
                         }
                         break;
                         
                     case DFP_FIND_VER_BYTE:
                         if (rec_char != DFP_VER) {
                             if (player->debug) {
                                 ESP_LOGW(TAG, "Version error: 0x%02X", rec_char);
                             }
                             player->state = DFP_FIND_START_BYTE;
                             return false;
                         }
                         player->rec_stack.version = rec_char;
                         player->state = DFP_FIND_LEN_BYTE;
                         break;
                         
                     case DFP_FIND_LEN_BYTE:
                         if (rec_char != DFP_LEN) {
                             if (player->debug) {
                                 ESP_LOGW(TAG, "Length error: 0x%02X", rec_char);
                             }
                             player->state = DFP_FIND_START_BYTE;
                             return false;
                         }
                         player->rec_stack.length = rec_char;
                         player->state = DFP_FIND_COMMAND_BYTE;
                         break;
                         
                     case DFP_FIND_COMMAND_BYTE:
                         player->rec_stack.commandValue = rec_char;
                         player->state = DFP_FIND_FEEDBACK_BYTE;
                         break;
                         
                     case DFP_FIND_FEEDBACK_BYTE:
                         player->rec_stack.feedbackValue = rec_char;
                         player->state = DFP_FIND_PARAM_MSB;
                         break;
                         
                     case DFP_FIND_PARAM_MSB:
                         player->rec_stack.paramMSB = rec_char;
                         player->state = DFP_FIND_PARAM_LSB;
                         break;
                         
                     case DFP_FIND_PARAM_LSB:
                         player->rec_stack.paramLSB = rec_char;
                         player->state = DFP_FIND_CHECKSUM_MSB;
                         break;
                         
                     case DFP_FIND_CHECKSUM_MSB:
                         player->rec_stack.checksumMSB = rec_char;
                         player->state = DFP_FIND_CHECKSUM_LSB;
                         break;
                         
                     case DFP_FIND_CHECKSUM_LSB:
                         player->rec_stack.checksumLSB = rec_char;
                         
                         // Store received checksum
                         uint16_t rec_checksum = (player->rec_stack.checksumMSB << 8) | player->rec_stack.checksumLSB;
                         
                         // Calculate checksum based on received data
                         dfplayer_stack_t temp_stack = player->rec_stack;
                         dfplayer_find_checksum(&temp_stack);
                         uint16_t calc_checksum = (temp_stack.checksumMSB << 8) | temp_stack.checksumLSB;
                         
                         if (rec_checksum != calc_checksum) {
                             if (player->debug) {
                                 ESP_LOGW(TAG, "Checksum error: received 0x%04X, calculated 0x%04X", rec_checksum, calc_checksum);
                             }
                             player->state = DFP_FIND_START_BYTE;
                             return false;
                         }
                         
                         player->state = DFP_FIND_END_BYTE;
                         break;
                         
                     case DFP_FIND_END_BYTE:
                         if (rec_char != DFP_EB) {
                             if (player->debug) {
                                 ESP_LOGW(TAG, "End byte error: 0x%02X", rec_char);
                             }
                             player->state = DFP_FIND_START_BYTE;
                             return false;
                         }
                         
                         player->rec_stack.end_byte = rec_char;
                         player->state = DFP_FIND_START_BYTE;
                         return true;
                         
                     default:
                         break;
                 }
             }
         } else {
             // Give other tasks a chance to run
             vTaskDelay(1);
         }
     }
     
     if (player->debug) {
         ESP_LOGW(TAG, "Timeout error");
     }
     
     player->state = DFP_FIND_START_BYTE;
     return false;
 }
 




 /**
  * @brief Clear the UART receive buffer
  * 
  * @param player Handle to the DFPlayer instance
  */
 static void dfplayer_flush(dfplayer_handle_t player) {
     uart_flush(player->uart_port);
 }
 

 /**
  * @brief Send a query to the DFPlayer and get the result
  * 
  * @param player Handle to the DFPlayer instance
  * @param cmd Command to send
  * @param msb Parameter MSB
  * @param lsb Parameter LSB
  * @return int16_t Query result, -1 on error
  */
 static int16_t dfplayer_query(dfplayer_handle_t player, uint8_t cmd, uint8_t msb, uint8_t lsb) {
     dfplayer_flush(player);
     
     player->send_stack.commandValue = cmd;
     player->send_stack.feedbackValue = DFP_NO_FEEDBACK;
     player->send_stack.paramMSB = msb;
     player->send_stack.paramLSB = lsb;
     
     dfplayer_find_checksum(&player->send_stack);
     
     if (dfplayer_send_data(player) != ESP_OK) {
         return -1;
     }
     
     if (dfplayer_parse_feedback(player)) {
         if (player->rec_stack.commandValue != 0x40) {
             return (player->rec_stack.paramMSB << 8) | player->rec_stack.paramLSB;
         }
     }
     
     return -1;
 }




/**
 * @brief High-level function to send commands to the DFPlayer
 * 
 * @param player Handle to the DFPlayer instance
 * @param cmd Command to send
 * @param feedback Feedback setting
 * @param msb Parameter MSB
 * @param lsb Parameter LSB
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t dfplayer_send_command(dfplayer_handle_t player, uint8_t cmd, uint8_t feedback, uint8_t msb, uint8_t lsb) {
    if (player == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    player->send_stack.commandValue = cmd;
    player->send_stack.feedbackValue = feedback;
    player->send_stack.paramMSB = msb;
    player->send_stack.paramLSB = lsb;
    
    dfplayer_find_checksum(&player->send_stack);
    return dfplayer_send_data(player);
}
 


 /**
  * @brief Print error message based on error code
  * 
  * @param player Handle to the DFPlayer instance
  */
 /*
 static void dfplayer_print_error(dfplayer_handle_t player) {
     if (player->rec_stack.commandValue == 0x40) {
         switch (player->rec_stack.paramLSB) {
             case 0x1:
                 ESP_LOGE(TAG, "Module busy (initialization not done)");
                 break;
             case 0x2:
                 ESP_LOGE(TAG, "Currently in sleep mode");
                 break;
             case 0x3:
                 ESP_LOGE(TAG, "Serial receiving error (incomplete frame)");
                 break;
             case 0x4:
                 ESP_LOGE(TAG, "Checksum incorrect");
                 break;
             case 0x5:
                 ESP_LOGE(TAG, "Specified track is out of current track scope");
                 break;
             case 0x6:
                 ESP_LOGE(TAG, "Specified track not found");
                 break;
             case 0x7:
                 ESP_LOGE(TAG, "Insertion error (insertion only works during playback)");
                 break;
             case 0x8:
                 ESP_LOGE(TAG, "SD card reading failed (card removed or damaged)");
                 break;
             case 0xA:
                 ESP_LOGI(TAG, "Entered into sleep mode");
                 break;
             default:
                 ESP_LOGE(TAG, "Unknown error: 0x%02X", player->rec_stack.paramLSB);
                 break;
         }
     } else {
         ESP_LOGI(TAG, "No error");
     }
 }
 */



 // Public API implementations
 esp_err_t dfplayer_deinit(dfplayer_handle_t player) {
     if (player == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     free(player);
     return ESP_OK;
 }
 



 dfplayer_handle_t dfplayer_init(dfplayer_config_t *config) {
     if (config == NULL) {
         ESP_LOGE(TAG, "Configuration is NULL");
         return NULL;
     }
     
     // Allocate memory for the handle
     dfplayer_handle_t player = calloc(1, sizeof(struct dfplayer_handle));
     if (player == NULL) {
         ESP_LOGE(TAG, "Failed to allocate memory for handle");
         return NULL;
     }
     
     player->uart_port = config->uart_port;
     player->debug = config->debug;
     player->timeout_ms = config->timeout_ms;
     player->state = DFP_FIND_START_BYTE;
     
     // Initialize stacks
     player->send_stack.start_byte = DFP_SB;
     player->send_stack.version = DFP_VER;
     player->send_stack.length = DFP_LEN;
     player->send_stack.end_byte = DFP_EB;
     
     player->rec_stack.start_byte = DFP_SB;
     player->rec_stack.version = DFP_VER;
     player->rec_stack.length = DFP_LEN;
     player->rec_stack.end_byte = DFP_EB;
     
     ESP_LOGI(TAG, "DFPlayer initialized, UART port %d", player->uart_port);
     
     return player;
 }
 


 esp_err_t dfplayer_play_next(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_NEXT, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_play_previous(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_PREV, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_play(dfplayer_handle_t player, uint16_t track_num) {
     return dfplayer_send_command(player, DFP_PLAY, DFP_NO_FEEDBACK, (track_num >> 8) & 0xFF, track_num & 0xFF);
 }
 
 esp_err_t dfplayer_stop(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_STOP, DFP_NO_FEEDBACK, 0, 0);
 }
 
 esp_err_t dfplayer_play_from_mp3_folder(dfplayer_handle_t player, uint16_t track_num) {
     return dfplayer_send_command(player, DFP_USE_MP3_FOLDER, DFP_NO_FEEDBACK, (track_num >> 8) & 0xFF, track_num & 0xFF);
 }
 
 esp_err_t dfplayer_play_folder(dfplayer_handle_t player, uint8_t folder_num, uint8_t track_num) {
     return dfplayer_send_command(player, DFP_SPEC_FOLDER, DFP_NO_FEEDBACK, folder_num, track_num);
 }
 
 esp_err_t dfplayer_play_large_folder(dfplayer_handle_t player, uint8_t folder_num, uint16_t track_num) {
     const uint16_t arg = (((uint16_t)folder_num) << 12) | (track_num & 0xfff);
     return dfplayer_send_command(player, DFP_SPEC_TRACK_3000, DFP_NO_FEEDBACK, arg >> 8, arg & 0xff);
 }
 
 esp_err_t dfplayer_volume(dfplayer_handle_t player, uint8_t volume) {
     if (volume > 30) {
         return ESP_ERR_INVALID_ARG;
     }
     return dfplayer_send_command(player, DFP_VOLUME, DFP_NO_FEEDBACK, 0, volume);
 }
 
 esp_err_t dfplayer_volume_up(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_INC_VOL, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_volume_down(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_DEC_VOL, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_eq_select(dfplayer_handle_t player, uint8_t eq) {
     if (eq > 5) {
         return ESP_ERR_INVALID_ARG;
     }
     return dfplayer_send_command(player, DFP_EQ, DFP_NO_FEEDBACK, 0, eq);
 }
 
 esp_err_t dfplayer_loop(dfplayer_handle_t player, uint16_t track_num) {
     return dfplayer_send_command(player, DFP_PLAYBACK_MODE, DFP_NO_FEEDBACK, (track_num >> 8) & 0xFF, track_num & 0xFF);
 }
 
 esp_err_t dfplayer_playback_source(dfplayer_handle_t player, uint8_t source) {
     if (source == 0 || source > 5) {
         return ESP_ERR_INVALID_ARG;
     }
     return dfplayer_send_command(player, DFP_PLAYBACK_SRC, DFP_NO_FEEDBACK, 0, source);
 }
 
 esp_err_t dfplayer_standby(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_STANDBY, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_normal_mode(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_NORMAL, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_reset(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_RESET, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_resume(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_PLAYBACK, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_pause(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_PAUSE, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_random_all(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_RANDOM_ALL, DFP_NO_FEEDBACK, 0, 0);
 }
 
 esp_err_t dfplayer_start_dac(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_SET_DAC, DFP_NO_FEEDBACK, 0, 0);
 }
 
 esp_err_t dfplayer_stop_dac(dfplayer_handle_t player) {
     return dfplayer_send_command(player, DFP_SET_DAC, DFP_NO_FEEDBACK, 0, 1);
 }
 
 esp_err_t dfplayer_sleep(dfplayer_handle_t player) {
     return dfplayer_playback_source(player, DFP_SLEEP);
 }
 

 esp_err_t dfplayer_wake_up(dfplayer_handle_t player) {
     return dfplayer_playback_source(player, DFP_TF);
 }
 

 esp_err_t dfplayer_is_playing(dfplayer_handle_t player, bool *is_playing) {
     if (player == NULL || is_playing == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_STATUS, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *is_playing = (result & 1) != 0;
     return ESP_OK;
 }
 
 esp_err_t dfplayer_get_volume(dfplayer_handle_t player, int16_t *volume) {
     if (player == NULL || volume == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_VOL, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *volume = result;
     return ESP_OK;
 }
 

 esp_err_t dfplayer_get_eq(dfplayer_handle_t player, int16_t *eq) {
     if (player == NULL || eq == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_EQ, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *eq = result;
     return ESP_OK;
 }
 

 esp_err_t dfplayer_get_mode(dfplayer_handle_t player, int16_t *mode) {
     if (player == NULL || mode == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_MODE, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *mode = result;
     return ESP_OK;
 }
 

 esp_err_t dfplayer_get_sd_track_count(dfplayer_handle_t player, int16_t *count) {
     if (player == NULL || count == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_TF_FILES, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *count = result;
     return ESP_OK;
 }
 

 esp_err_t dfplayer_get_current_sd_track(dfplayer_handle_t player, int16_t *track) {
     if (player == NULL || track == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
     
     int16_t result = dfplayer_query(player, DFP_GET_TF_TRACK, 0, 0);
     if (result == -1) {
         return ESP_FAIL;
     }
     
     *track = result;
     return ESP_OK;
 }