/**
 * vl53l1_platform.c
 * 
 * Platform-specific interface functions for the ST VL53L1 family of
 * time-of-flight laser ranging sensors, specific implementation for the
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
 * 
 * License terms: BSD 3-clause "New" or "Revised" License. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this 
 * list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 * may be used to endorse or promote products derived from this software 
 * without specific prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 * 
 */


 #include "esp_timer.h"
 #include "VL53L1X_api.h"
 #include "esp_log.h"
 #include <inttypes.h>
 #include "i2c_platform_esp.h"
 #include "sdkconfig.h"


 static const char *TAG = "PLATFORM";
 

 extern i2c_master_dev_handle_t i2c_handle; 


 static const uint8_t status_rtn[24] = { 
     255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
     255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
     255, 255, 11, 12
 };
 



 /**
  * vl53l1 platform-specific implementation, I2C read-write device functions
  */
 
 VL53L1X_ERROR esp_to_vl53l1x_error(esp_err_t esp_code){
     VL53L1X_ERROR vl_error;
     
     switch(esp_code) {
     case ESP_OK:                    vl_error = VL53L1_ERROR_NONE; break;
     case ESP_FAIL:                  vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_NO_MEM:            vl_error = VL53L1_ERROR_BUFFER_TOO_SMALL; break;
     case ESP_ERR_INVALID_ARG:       vl_error = VL53L1_ERROR_INVALID_PARAMS; break;
     case ESP_ERR_INVALID_STATE:     vl_error = VL53L1_ERROR_INVALID_COMMAND; break;
     case ESP_ERR_INVALID_SIZE:      vl_error = VL53L1_ERROR_INVALID_COMMAND; break;
     case ESP_ERR_NOT_FOUND:         vl_error = VL53L1_ERROR_NOT_SUPPORTED; break;
     case ESP_ERR_NOT_SUPPORTED:     vl_error = VL53L1_ERROR_NOT_SUPPORTED; break;
     case ESP_ERR_TIMEOUT:           vl_error = VL53L1_ERROR_TIME_OUT; break;
     case ESP_ERR_INVALID_RESPONSE:  vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_INVALID_CRC:       vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_INVALID_VERSION:   vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_INVALID_MAC:       vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_WIFI_BASE:         vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_MESH_BASE:         vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     case ESP_ERR_FLASH_BASE:        vl_error = VL53L1_ERROR_CONTROL_INTERFACE; break;
     default:                        vl_error = VL53L1_ERROR_UNDEFINED; break;
     }
     
     if (vl_error != VL53L1_ERROR_NONE) {
         ESP_LOGW(TAG, "ESP error code %d converted to VL53L1X error %d", esp_code, vl_error);
     }
     
     return vl_error;
 }
 

 




 VL53L1X_ERROR VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    
    ESP_LOGV(TAG, "Writing %" PRIu32 " bytes to device 0x%02X at index 0x%04X", count, dev, index);

    if (I2C_Master == NULL || I2C_Master->dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }

    // Creazione del buffer per l'indirizzo e i dati
    uint8_t buffer[count + 2];  
    buffer[0] = (index >> 8) & 0xFF;  // MSB dell'indirizzo
    buffer[1] = index & 0xFF;         // LSB dell'indirizzo
    memcpy(&buffer[2], pdata, count); // Copia i dati nel buffer

    // Scrive i dati al dispositivo VL53L1X
    esp_err_t err = i2c_master_transmit(I2C_Master->dev_handle, buffer, count + 2, -1);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to device 0x%02X: %s", dev, esp_err_to_name(err));
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }

    ESP_LOGV(TAG, "Successfully wrote %" PRIu32 " bytes to device 0x%02X at index 0x%04X", count, dev, index);
    
    return VL53L1_ERROR_NONE;
}


 


 VL53L1X_ERROR VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
     //ESP_LOGV(TAG, "Writing byte 0x%02X to device 0x%02X at index 0x%04X", data, dev, index);    
     int status;
     uint8_t write_data = data;
     status = VL53L1_WriteMulti(dev, index, &write_data, 1);
     return status;
 }
 



 VL53L1X_ERROR VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
     ESP_LOGV(TAG, "Writing word 0x%04X to device 0x%02X at index 0x%04X", data, dev, index);   
     int status;
     uint8_t buffer[2];
     buffer[0] = data >> 8;
     buffer[1] = data & 0x00FF;
     status = VL53L1_WriteMulti(dev, index, (uint8_t *)buffer, 2);
     return status;
 }
 

 

 VL53L1X_ERROR VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
     ESP_LOGV(TAG, "Writing dword 0x%08"PRIx32" to device 0x%02X at index 0x%04X", data, dev, index);   
     int status;
     uint8_t buffer[4];
     buffer[0] = (data >> 24) & 0xFF;
     buffer[1] = (data >> 16) & 0xFF;
     buffer[2] = (data >>  8) & 0xFF;
     buffer[3] = (data >>  0) & 0xFF;
     status = VL53L1_WriteMulti(dev, index, (uint8_t *)buffer, 4);
     return status;
 }
 


 VL53L1X_ERROR VL53L1_UpdateByte(uint16_t dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
     ESP_LOGV(TAG, "Updating byte at device 0x%02X, index 0x%04X with AND 0x%02X, OR 0x%02X", dev, index, AndData, OrData);
     
     int status;
     uint8_t buffer = 0;
 
     /* read data direct onto buffer */
     status = VL53L1_ReadMulti(dev, index, &buffer, 1);
     if (status) {
         ESP_LOGE(TAG, "Error reading byte during update operation: %d", status);
         return status;
     }
     
     uint8_t original = buffer;
     buffer = (buffer & AndData) | OrData;
     
     ESP_LOGV(TAG, "Updating byte value from 0x%02X to 0x%02X", original, buffer);
     
     status = VL53L1_WriteMulti(dev, index, &buffer, (uint16_t)1);
     return status;
 }
 






 VL53L1X_ERROR VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data){

    int status = 0; // 0 = successo, !=0 = errore

    //ESP_LOGI(TAG, "Leggo un byte dal device 0x%02X al registro 0x%04X", dev, index);

    // 1) Avvia la comunicazione in scrittura
    if (i2c_start(dev, I2C_WRITE))
    {
        //ESP_LOGI(TAG, "Device handle created successfully");

        // 2) Prepara i 2 byte di indirizzo di registro
        uint8_t reg_addr[2] = {(uint8_t)(index >> 8), (uint8_t)(index & 0x00FF)}; // MSB prima di LSB

        // 3) Invia l'indirizzo del registro
        status = i2c_write(reg_addr, 2);
        ESP_LOGV(TAG, "Register address write status: %d", status);

        // 4) Completa la transazione di scrittura
        i2c_transmit();

        // 5) Piccolo ritardo
        vTaskDelay(pdMS_TO_TICKS(5));

        // 6) Riavvia in modalità lettura
        if (i2c_start(dev, I2C_READ))
        {
            // 7) Leggi il byte
            *data = i2c_read_byte();

            // 8) Completa la transazione
            i2c_transmit();

            //ESP_LOGI(TAG, "Byte letto: 0x%02X", *data);
            return 0; // Successo
        }
        else
        {
            ESP_LOGE(TAG, "Failed to restart communication in READ mode");
            return -1; // Errore
        }

    }else{
        ESP_LOGE(TAG, "Impossibile aprire la comunicazione in modalita' SCRITTURA");
        return -1; // Errore
    }
}




 VL53L1X_ERROR VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
     int status;
     uint8_t buffer[2] = {0, 0};
     status = VL53L1_ReadMulti(dev, index, buffer, 2);
     if (status) return status;
     
     *data = (buffer[0] << 8) + buffer[1];
     ESP_LOGV(TAG, "Read word 0x%04X from device 0x%02X at index 0x%04X", *data, dev, index);
     
     return status;
 }
 


 VL53L1X_ERROR VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
     int status;
     uint8_t buffer[4] = {0, 0, 0, 0};
     status = VL53L1_ReadMulti(dev, index, buffer, 4);
     if (status) return status;
     
     *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
     ESP_LOGV(TAG, "Read dword 0x%08"PRIx32" from device 0x%02X at index 0x%04X", *data, dev, index);
     
     return status;
 }
 

 
/**
  * vl53l1 platform-specific implementation, O/S timing functions
*/
 VL53L1X_ERROR VL53L1_GetTickCount(uint32_t *ptick_count_ms){
     // note: _ms means microseconds, not milliseconds
     *ptick_count_ms = esp_timer_get_time();
     ESP_LOGV(TAG, "Get tick count: %"PRIu32" µs", *ptick_count_ms);
     return VL53L1_ERROR_NONE;
 }
 


 VL53L1X_ERROR VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz){
     *ptimer_freq_hz = CONFIG_VL53L1_I2C_DEFAULT_FREQ;
     ESP_LOGV(TAG, "Get timer frequency: %"PRId32" Hz", *ptimer_freq_hz);
     return VL53L1_ERROR_NONE;
 }
 

 VL53L1X_ERROR VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
     int32_t actual_wait = MAX(1, wait_ms / portTICK_PERIOD_MS);
     ESP_LOGV(TAG, "Waiting %"PRId32" ms (actual ticks: %"PRId32")", wait_ms, actual_wait);
     vTaskDelay(actual_wait);
     return VL53L1_ERROR_NONE;
 }
 



 VL53L1X_ERROR VL53L1_WaitUs(uint16_t dev, int32_t wait_us){
     int32_t actual_wait = MAX(1, wait_us / 1000 / portTICK_PERIOD_MS);
     ESP_LOGV(TAG, "Waiting %"PRId32" µs (actual ticks: %"PRId32")", wait_us, actual_wait);
     vTaskDelay(actual_wait);
     return VL53L1_ERROR_NONE;
 }
 


 // other useful device functions
 VL53L1X_ERROR VL53L1X_SetFastI2C(uint16_t dev){
     ESP_LOGI(TAG, "Setting fast I2C for device 0x%02X", dev);
     return VL53L1_WrByte(dev, VL53L1_PAD_I2C_HV__CONFIG, 0x14);
 }
 
 /**
  * 	fields: \n
  *		- [1:0] = scheduler_mode
  *		- [3:2] = readout_mode
  *		-   [4] = mode_range__single_shot
  *		-   [5] = mode_range__back_to_back
  *		-   [6] = mode_range__timed
  *		-   [7] = mode_range__abort 
  */
 VL53L1X_ERROR VL53L1X_SetRangingMode(uint16_t dev, uint8_t set_ranging_mode){
     ESP_LOGI(TAG, "Setting ranging mode 0x%02X for device 0x%02X", set_ranging_mode, dev);
     
     uint8_t mode_start;
     VL53L1_RdByte(dev, VL53L1_SYSTEM__MODE_START, &mode_start);
     mode_start = (mode_start & 0x0F) | set_ranging_mode;
     
     ESP_LOGD(TAG, "Updated mode_start value: 0x%02X", mode_start);
     
     return VL53L1_WrByte(dev, VL53L1_SYSTEM__MODE_START, mode_start);
 }
 


 VL53L1X_ERROR VL53L1X_SystemStatus(uint16_t dev, uint8_t *state){
     VL53L1X_ERROR status = VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, state);
     
     if (status == VL53L1_ERROR_NONE) {
         ESP_LOGI(TAG, "Device 0x%02X system status: 0x%02X", dev, *state);
     } else {
         ESP_LOGE(TAG, "Failed to read system status from device 0x%02X, error: %d", dev, status);
     }
     
     return status;
 }
 



 char *VL53L1X_SystemStatusString(uint16_t dev){
     uint8_t system_status;
     VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &system_status);
     
     char *status_str;
     
     switch(system_status) {
     case VL53L1_STATE_POWERDOWN:        status_str = "VL53L1_STATE_POWERDOWN"; break;
     case VL53L1_STATE_WAIT_STATICINIT:  status_str = "VL53L1_STATE_WAIT_STATICINIT"; break;
     case VL53L1_STATE_STANDBY:          status_str = "VL53L1_STATE_STANDBY"; break;
     case VL53L1_STATE_IDLE:             status_str = "VL53L1_STATE_IDLE"; break;
     case VL53L1_STATE_RUNNING:          status_str = "VL53L1_STATE_RUNNING"; break;
     case VL53L1_STATE_RESET:            status_str = "VL53L1_STATE_RESET"; break;
     case VL53L1_STATE_UNKNOWN:          status_str = "VL53L1_STATE_UNKNOWN"; break;
     case VL53L1_STATE_ERROR:            status_str = "VL53L1_STATE_ERROR"; break;
     default:                            status_str = "VL53L1_STATE_UNKNOWN"; break;
     }
     
     ESP_LOGI(TAG, "Device 0x%02X system status: %s", dev, status_str);
     return status_str;
 }
 


 VL53L1X_ERROR VL53L1X_GetContinuousMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM){
     VL53L1X_ERROR status;
 
     // VL53L1X_GetRangeStatus(dev, &RangeStatus)
     uint8_t RgSt;
     status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
     *rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;
 
     //	VL53L1X_GetDistance(dev, &Distance)
     VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);
 
     //  VL53L1X_ClearInterrupt(dev)
     VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
     
     //ESP_LOGI(TAG, "Device 0x%02X continuous measurement: distance=%u mm, status=%u", dev, *distanceMM, *rangeStatus);
 
     return status;
 }
 


 VL53L1X_ERROR VL53L1X_GetAndRestartMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM){
     VL53L1X_ERROR status;
 
     // VL53L1X_GetRangeStatus(dev, &RangeStatus)
     uint8_t RgSt;
     status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
     *rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;
 
     //	VL53L1X_GetDistance(dev, &Distance)
     VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);
 
     ESP_LOGI(TAG, "Device 0x%02X measurement: distance=%u mm, status=%u", dev, *distanceMM, *rangeStatus);
              
     //  VL53L1X_StartRanging(dev)
     ESP_LOGI(TAG, "Restarting ranging for device 0x%02X", dev);
     VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);
 
     return status;
 }
 




 VL53L1X_ERROR VL53L1X_InitSensorArray(VL53L1_DEV sensor_array, uint8_t sensor_count){
     ESP_LOGI(TAG, "Initializing array of %u VL53L1X sensors", sensor_count);
     
     uint8_t sensorState = 0;
     uint16_t timeout_check = 0;
     int k;
 
     // shut off all sensors and initialize sensor structures
     ESP_LOGI(TAG, "Shutting down all sensors and initializing structures");
     for (k = 0; k < sensor_count; k++) {
         pinMode(sensor_array[k].shutdown_pin, OUTPUT_OPEN);
         digitalWrite(sensor_array[k].shutdown_pin, LOW);
         sensor_array[k].time_stamp = esp_timer_get_time();
         sensor_array[k].cycle_time = 0;
         sensor_array[k].range_mm = 0;
         sensor_array[k].range_status = 0;
         sensor_array[k].range_error = VL53L1_ERROR_NONE;
         
         ESP_LOGI(TAG, "Sensor %d: shutdown pin=%d, I2C address=0x%02X",  k, sensor_array[k].shutdown_pin, sensor_array[k].I2cDevAddr);
     }
     vTaskDelay(100 / portTICK_PERIOD_MS);
 

     for (k = 0; k < sensor_count; k++) {
         ESP_LOGI(TAG, "Initializing sensor %d", k);
         
         // enable this device and wait for it to boot up
         ESP_LOGI(TAG, "Enabling sensor %d and waiting for boot", k);

         digitalWrite(sensor_array[k].shutdown_pin, HIGH);
         timeout_check = sensorState = 0;

         while (sensorState == 0) {
             vTaskDelay(20 / portTICK_PERIOD_MS);

             VL53L1X_BootState(CONFIG_VL53L1_I2C_ADDRESS, &sensorState);

             if (++timeout_check > 10) {
                 ESP_LOGE(TAG, "Timeout waiting for sensor %d to boot", k);
                 return VL53L1_ERROR_TIME_OUT;
             }
         }
         
         ESP_LOGI(TAG, "Sensor %d booted successfully", k);
 
         // initialize the device
         ESP_LOGI(TAG, "Initializing sensor %d core functions", k);
         VL53L1X_SensorInit(CONFIG_VL53L1_I2C_ADDRESS);
 
         // change it's I2C address and use that new I2C address from now on
         ESP_LOGI(TAG, "Changing sensor %d I2C address from 0x%02X to 0x%02X", k, CONFIG_VL53L1_I2C_ADDRESS, sensor_array[k].I2cDevAddr);
         VL53L1X_SetI2CAddress(CONFIG_VL53L1_I2C_ADDRESS, sensor_array[k].I2cDevAddr);
 
         // configure the device
         ESP_LOGI(TAG, "Configuring sensor %d parameters", k);
         VL53L1X_SetFastI2C(sensor_array[k].I2cDevAddr);
         VL53L1X_SetDistanceMode(sensor_array[k].I2cDevAddr, sensor_array[k].distance_mode);
         ESP_LOGI(TAG, "Sensor %d: distance mode=%d", k, sensor_array[k].distance_mode);
         
         VL53L1X_SetTimingBudgetInMs(sensor_array[k].I2cDevAddr, sensor_array[k].timing_budget);
         ESP_LOGI(TAG, "Sensor %d: timing budget=%d ms", k, sensor_array[k].timing_budget);
         
         VL53L1X_SetInterMeasurementInMs(sensor_array[k].I2cDevAddr, sensor_array[k].inter_measurement);
         ESP_LOGI(TAG, "Sensor %d: inter-measurement=%d ms", k, sensor_array[k].inter_measurement);
         
         VL53L1X_SetRangingMode(sensor_array[k].I2cDevAddr, RANGING_MODE_SINGLE_SHOT);
 
         // kick off measurement cycle
         ESP_LOGI(TAG, "Starting ranging for sensor %d", k);
         VL53L1X_StartRanging(sensor_array[k].I2cDevAddr);
     }
     vTaskDelay(100 / portTICK_PERIOD_MS);
     
     ESP_LOGI(TAG, "Successfully initialized %d VL53L1X sensors", sensor_count);
 
     return VL53L1_ERROR_NONE;
 }