/*
 Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file : part of VL53L1 Core and : dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

/**
 * @file  vl53l1x_api.c
 * @brief Functions implementation
 */

 #include "VL53L1X_api.h"
 #include <string.h>
 #include "esp_log.h"
 #include <inttypes.h>
 #include "vl53l1_platform.h"

 static const char *TAG = "VL53L1X_API";
 
 #if 0
 uint8_t VL51L1X_NVM_CONFIGURATION[] = {
 0x00, /* 0x00 : not user-modifiable */
 0x29, /* 0x01 : 7 bits I2C address (default=0x29), use SetI2CAddress(). Warning: after changing the register value to a new I2C address, the device will only answer to the new address */
 0x00, /* 0x02 : not user-modifiable */
 0x00, /* 0x03 : not user-modifiable */
 0x00, /* 0x04 : not user-modifiable */
 0x00, /* 0x05 : not user-modifiable */
 0x00, /* 0x06 : not user-modifiable */
 0x00, /* 0x07 : not user-modifiable */
 0x00, /* 0x08 : not user-modifiable */
 0x50, /* 0x09 : not user-modifiable */
 0x00, /* 0x0A : not user-modifiable */
 0x00, /* 0x0B : not user-modifiable */
 0x00, /* 0x0C : not user-modifiable */
 0x00, /* 0x0D : not user-modifiable */
 0x0a, /* 0x0E : not user-modifiable */
 0x00, /* 0x0F : not user-modifiable */
 0x00, /* 0x10 : not user-modifiable */
 0x00, /* 0x11 : not user-modifiable */
 0x00, /* 0x12 : not user-modifiable */
 0x00, /* 0x13 : not user-modifiable */
 0x00, /* 0x14 : not user-modifiable */
 0x00, /* 0x15 : not user-modifiable */
 0x00, /* 0x16 : Xtalk calibration value MSB (7.9 format in kcps), use SetXtalk() */
 0x00, /* 0x17 : Xtalk calibration value LSB */
 0x00, /* 0x18 : not user-modifiable */
 0x00, /* 0x19 : not user-modifiable */
 0x00, /* 0x1a : not user-modifiable */
 0x00, /* 0x1b : not user-modifiable */
 0x00, /* 0x1e : Part to Part offset x4 MSB (in mm), use SetOffset() */
 0x50, /* 0x1f : Part to Part offset x4 LSB */
 0x00, /* 0x20 : not user-modifiable */
 0x00, /* 0x21 : not user-modifiable */
 0x00, /* 0x22 : not user-modifiable */
 0x00, /* 0x23 : not user-modifiable */
 }
 #endif
 
 const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
 0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
 0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
 0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
 0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
 0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
 0x00, /* 0x32 : not user-modifiable */
 0x02, /* 0x33 : not user-modifiable */
 0x08, /* 0x34 : not user-modifiable */
 0x00, /* 0x35 : not user-modifiable */
 0x08, /* 0x36 : not user-modifiable */
 0x10, /* 0x37 : not user-modifiable */
 0x01, /* 0x38 : not user-modifiable */
 0x01, /* 0x39 : not user-modifiable */
 0x00, /* 0x3a : not user-modifiable */
 0x00, /* 0x3b : not user-modifiable */
 0x00, /* 0x3c : not user-modifiable */
 0x00, /* 0x3d : not user-modifiable */
 0xff, /* 0x3e : not user-modifiable */
 0x00, /* 0x3f : not user-modifiable */
 0x0F, /* 0x40 : not user-modifiable */
 0x00, /* 0x41 : not user-modifiable */
 0x00, /* 0x42 : not user-modifiable */
 0x00, /* 0x43 : not user-modifiable */
 0x00, /* 0x44 : not user-modifiable */
 0x00, /* 0x45 : not user-modifiable */
 0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
 0x0b, /* 0x47 : not user-modifiable */
 0x00, /* 0x48 : not user-modifiable */
 0x00, /* 0x49 : not user-modifiable */
 0x02, /* 0x4a : not user-modifiable */
 0x0a, /* 0x4b : not user-modifiable */
 0x21, /* 0x4c : not user-modifiable */
 0x00, /* 0x4d : not user-modifiable */
 0x00, /* 0x4e : not user-modifiable */
 0x05, /* 0x4f : not user-modifiable */
 0x00, /* 0x50 : not user-modifiable */
 0x00, /* 0x51 : not user-modifiable */
 0x00, /* 0x52 : not user-modifiable */
 0x00, /* 0x53 : not user-modifiable */
 0xc8, /* 0x54 : not user-modifiable */
 0x00, /* 0x55 : not user-modifiable */
 0x00, /* 0x56 : not user-modifiable */
 0x38, /* 0x57 : not user-modifiable */
 0xff, /* 0x58 : not user-modifiable */
 0x01, /* 0x59 : not user-modifiable */
 0x00, /* 0x5a : not user-modifiable */
 0x08, /* 0x5b : not user-modifiable */
 0x00, /* 0x5c : not user-modifiable */
 0x00, /* 0x5d : not user-modifiable */
 0x01, /* 0x5e : not user-modifiable */
 0xcc, /* 0x5f : not user-modifiable */
 0x0f, /* 0x60 : not user-modifiable */
 0x01, /* 0x61 : not user-modifiable */
 0xf1, /* 0x62 : not user-modifiable */
 0x0d, /* 0x63 : not user-modifiable */
 0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
 0x68, /* 0x65 : Sigma threshold LSB */
 0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
 0x80, /* 0x67 : Min count Rate LSB */
 0x08, /* 0x68 : not user-modifiable */
 0xb8, /* 0x69 : not user-modifiable */
 0x00, /* 0x6a : not user-modifiable */
 0x00, /* 0x6b : not user-modifiable */
 0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
 0x00, /* 0x6d : Intermeasurement period */
 0x0f, /* 0x6e : Intermeasurement period */
 0x89, /* 0x6f : Intermeasurement period LSB */
 0x00, /* 0x70 : not user-modifiable */
 0x00, /* 0x71 : not user-modifiable */
 0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
 0x00, /* 0x73 : distance threshold high LSB */
 0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
 0x00, /* 0x75 : distance threshold low LSB */
 0x00, /* 0x76 : not user-modifiable */
 0x01, /* 0x77 : not user-modifiable */
 0x0f, /* 0x78 : not user-modifiable */
 0x0d, /* 0x79 : not user-modifiable */
 0x0e, /* 0x7a : not user-modifiable */
 0x0e, /* 0x7b : not user-modifiable */
 0x00, /* 0x7c : not user-modifiable */
 0x00, /* 0x7d : not user-modifiable */
 0x02, /* 0x7e : not user-modifiable */
 0xc7, /* 0x7f : ROI center, use SetROI() */
 0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
 0x9B, /* 0x81 : not user-modifiable */
 0x00, /* 0x82 : not user-modifiable */
 0x00, /* 0x83 : not user-modifiable */
 0x00, /* 0x84 : not user-modifiable */
 0x01, /* 0x85 : not user-modifiable */
 0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
 0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
 };
 
 static const uint8_t status_rtn[24] = { 
	 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	 255, 255, 11, 12
 };
 


 VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion){
	VL53L1X_ERROR Status = 0;

	pVersion->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
	pVersion->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
	pVersion->build = VL53L1X_IMPLEMENTATION_VER_SUB;
	pVersion->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
	
	ESP_LOGI(TAG, "SW Version: %"PRIu32".%"PRIu32".%"PRIu32".%"PRIu32, (uint32_t)pVersion->major, (uint32_t)pVersion->minor, (uint32_t)pVersion->build, (uint32_t)pVersion->revision);
	        
	return Status;
}
 

VL53L1X_ERROR VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address){
	 VL53L1X_ERROR status = 0;
	 
	 ESP_LOGI(TAG, "Setting I2C address for device 0x%02X to new address 0x%02X", dev, new_address);
 
	 status = VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
	 
	 if (status == 0) {
		 ESP_LOGI(TAG, "I2C address successfully updated");
	 } else {
		 ESP_LOGE(TAG, "Failed to set I2C address, error: %d", status);
	 }
	 
	 return status;
 }
 





 


VL53L1X_ERROR VL53L1X_SensorInit(uint16_t dev) {

    VL53L1X_ERROR status = 0;
    uint8_t Addr = 0x00, tmp;
    

    // Reset hardware tramite XSHUT prima dell'inizializzazione
    ESP_LOGI(TAG, "Esecuzione reset hardware tramite XSHUT...");
    vl53l1x_hw_reset();
    
    // Verifica che il sensore risponda dopo il reset
    esp_err_t probe_ret = i2c_master_probe(I2C_Master->bus_handle, dev, 100);
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensore VL53L1X non risponde dopo reset: %s", esp_err_to_name(probe_ret));
        return -1;
    }
    ESP_LOGI(TAG, "Sensore VL53L1X rilevato sul bus I2C, indirizzo: 0x%02X", dev);
    
    // Salva la frequenza corrente dell'I2C per ripristinarla dopo
    uint32_t original_freq = I2C_Master->freq;
    if (original_freq > 100000) {
        ESP_LOGI(TAG, "Riduzione temporanea della frequenza I2C a 100kHz per l'inizializzazione");
        i2c_upgrade(100000);  // Riduce a 100kHz durante l'init per maggiore stabilità
    }
    
    // Verifica dell'ID del sensore prima di procedere
    uint16_t sensor_id = 0;
    status = VL53L1X_GetSensorId(dev, &sensor_id);
    if (status != 0) {
        ESP_LOGE(TAG, "Errore nella lettura dell'ID del sensore: %d", status);
        if (original_freq > 100000) i2c_upgrade(original_freq);
        return -1;
    }
    
    // 0xEACC è l'ID standard, ma potresti dover aggiornare questo valore secondo la documentazione
    if (sensor_id != 0xEACC) {
        ESP_LOGW(TAG, "ID del sensore non standard: 0x%04X (atteso 0xEACC)", sensor_id);
        // Continuiamo comunque, poiché alcune versioni del sensore potrebbero avere ID diversi
    } else {
        ESP_LOGI(TAG, "ID del sensore verificato: 0x%04X", sensor_id);
    }
    
    // Attesa aggiuntiva prima di iniziare la configurazione
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Primo passo: configura tutti i registri senza verifica immediata
    ESP_LOGI(TAG, "Inizio configurazione registri...");
    for (Addr = 0x2D; Addr <= 0x87; Addr++) {
        uint8_t written_value = VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D];
        
        status = VL53L1_WrByte(dev, Addr, written_value);
        if (status != 0) {
            ESP_LOGE(TAG, "Errore nella scrittura del registro 0x%02X", Addr);
            // Prova un nuovo tentativo invece di continuare
            vTaskDelay(pdMS_TO_TICKS(5));
            status = VL53L1_WrByte(dev, Addr, written_value);
            if (status != 0) {
                ESP_LOGE(TAG, "Secondo errore nella scrittura del registro 0x%02X, continuo con il prossimo", Addr);
                continue;
            }
        }
        
        // Attendi che il sensore elabori la scrittura (aumentato a 2ms)
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Pausa più lunga dopo aver scritto tutti i registri
    vTaskDelay(pdMS_TO_TICKS(30));  // Aumentato a 30ms per dare più tempo al sensore

    // Lista dei registri che storicamente danno problemi
    // Aggiungi qui altri registri problematici che hai identificato
    uint8_t critical_registers[] = {0x31, 0x44, 0x45, 0x65, 0x66, 0x67};
    int num_critical = sizeof(critical_registers) / sizeof(critical_registers[0]);

    // Verifica solo i registri critici
    ESP_LOGI(TAG, "Verifica dei registri critici...");
    for (int i = 0; i < num_critical; i++) {
        Addr = critical_registers[i];
        uint8_t expected_value = VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D];
        uint8_t read_value = 0;
        
        // Leggi il valore corrente
        status = VL53L1_RdByte(dev, Addr, &read_value);
        if (status != 0) {
            ESP_LOGE(TAG, "Errore nella lettura del registro critico 0x%02X", Addr);
            continue;
        }
        
        // Se non corrisponde, prova a riscriverlo con più tentativi
        if (read_value != expected_value) {
            ESP_LOGW(TAG, "Mismatch nel registro critico 0x%02X: atteso 0x%02X, letto 0x%02X - tentativo correzione", 
                     Addr, expected_value, read_value);
            
            // Prova fino a 3 volte per i registri critici
            for (int retry = 0; retry < 3; retry++) {
                // Reset del bus I2C prima di riprovare se non è il primo tentativo
                if (retry > 0) {
                    ESP_LOGW(TAG, "Reset del bus I2C prima del tentativo %d", retry+1);
                    i2c_master_bus_reset(I2C_Master->bus_handle);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                status = VL53L1_WrByte(dev, Addr, expected_value);
                if (status != 0) {
                    ESP_LOGE(TAG, "Errore tentativo %d di scrittura registro 0x%02X", retry+1, Addr);
                    continue;
                }
                
                // Attesa più lunga
                vTaskDelay(pdMS_TO_TICKS(10));  // Aumentato a 10ms
                
                // Verifica
                status = VL53L1_RdByte(dev, Addr, &read_value);
                if (status != 0) {
                    ESP_LOGE(TAG, "Errore tentativo %d di rilettura registro 0x%02X", retry+1, Addr);
                    continue;
                }
                
                if (read_value == expected_value) {
                    ESP_LOGI(TAG, "Registro 0x%02X corretto al tentativo %d", Addr, retry+1);
                    break;
                }
                
                // Se siamo all'ultimo tentativo e ancora non corrisponde
                if (retry == 2 && read_value != expected_value) {
                    ESP_LOGW(TAG, "Impossibile correggere il registro 0x%02X: atteso 0x%02X, letto 0x%02X",  Addr, expected_value, read_value);     
                    // Eccezioni note per registri specifici
                    if (Addr == 0x31) {
                        ESP_LOGW(TAG, "Il registro 0x31 mostra un valore persistente diverso. ""Questo potrebbe essere normale per alcune versioni del sensore.");
                    }
                }
            }
			
        } else {
            ESP_LOGI(TAG, "Registro critico 0x%02X verificato OK", Addr);
        }
    }

    ESP_LOGI(TAG, "Configurazione predefinita applicata");

    // Il resto della funzione rimane invariato
    status = VL53L1X_StartRanging(dev);
    ESP_LOGI(TAG, "Avvio del ciclo di ranging iniziale");
    
    // Attesa più lunga per il primo ciclo di ranging
    vTaskDelay(pdMS_TO_TICKS(20));
    
    tmp = 0;
    int timeout_counter = 0;
    while(tmp == 0) {
        status = VL53L1X_CheckForDataReady(dev, &tmp);
        vTaskDelay(pdMS_TO_TICKS(5));
        
        // Aggiungi un timeout per evitare loop infiniti
        timeout_counter++;
        if (timeout_counter > 100) {  // 500ms di timeout
            ESP_LOGW(TAG, "Timeout nell'attesa dei dati del primo ciclo di ranging");
            break;
        }
    }
    
    ESP_LOGI(TAG, "Ciclo di ranging iniziale completato");
    
    status = VL53L1X_ClearInterrupt(dev);
    status = VL53L1X_StopRanging(dev);
    
    status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
    status = VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
    


    // Ripristina la frequenza originale dell'I2C
    if (original_freq > 100000) {

		ESP_LOGI(TAG, "Ripristino della frequenza I2C originale: %"PRIu32" Hz", original_freq);

		i2c_upgrade(original_freq);
		
		// Verifica che il sensore sia ancora raggiungibile alla nuova frequenza
		esp_err_t probe_ret = i2c_master_probe(I2C_Master->bus_handle, dev, 100);
		if (probe_ret != ESP_OK) {
			ESP_LOGW(TAG, "Sensore non risponde dopo ripristino frequenza a %"PRIu32" Hz: %s", original_freq, esp_err_to_name(probe_ret));
			// Possiamo decidere di ridurre nuovamente la frequenza
			ESP_LOGI(TAG, "Riduzione frequenza a 100kHz per garantire stabilità");
			i2c_upgrade(100000);
		} else {
			ESP_LOGI(TAG, "Sensore risponde correttamente alla frequenza di %"PRIu32" Hz", original_freq);
		}

	}
    



    if (status == 0) {
		ESP_LOGI(TAG, "Inizializzazione del sensore VL53L1X completata con SUCCESSO (stato: 0)");
	} else {
		ESP_LOGE(TAG, "Inizializzazione del sensore VL53L1X FALLITA con errore (stato: %d)", status);
	}
    

    
    return status;
}






// Funzione per il reset hardware del sensore tramite XSHUT
void vl53l1x_hw_reset(void) {
    // Configura il pin XSHUT come output se non è già configurato
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_VL53L1_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Esegui reset hardware
    gpio_set_level(CONFIG_VL53L1_XSHUT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));  // Reset di 20ms
    gpio_set_level(CONFIG_VL53L1_XSHUT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));   // Attendi 50ms per startup completo
    
    ESP_LOGI(TAG, "Reset hardware completato tramite pin XSHUT (%d)", CONFIG_VL53L1_XSHUT_PIN);
}



// Funzione di supporto per il reset e reinizializzazione
bool vl53l1x_reset_and_init(uint16_t dev) {
    ESP_LOGI(TAG, "Esecuzione reset hardware e reinizializzazione VL53L1X");
    
    // Reset hardware
    vl53l1x_hw_reset();
    
    // Verifica che il sensore risponda
    ESP_LOGI(TAG, "Verifica presenza sensore dopo reset");
    i2c_scan();
    
    // Reinizializza il sensore
    VL53L1X_ERROR status = VL53L1X_SensorInit(dev);
    
    if (status != 0) {
        ESP_LOGE(TAG, "Fallita reinizializzazione del sensore dopo reset: %d", status);
        return false;
    }
    
    ESP_LOGI(TAG, "Reset e reinizializzazione completati con successo");
    return true;
}









 VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t dev){
	 VL53L1X_ERROR status = 0;
	 
	 //ESP_LOGD(TAG, "Clearing interrupt for device 0x%02X", dev);
	 
	 status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
	 
	 return status;
 }
 


 VL53L1X_ERROR VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t NewPolarity){
	 uint8_t Temp;
	 VL53L1X_ERROR status = 0;
	 
	 //ESP_LOGI(TAG, "Setting interrupt polarity for device 0x%02X to %d", dev, NewPolarity);
 
	 status = VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
	 Temp = Temp & 0xEF;
	 status = VL53L1_WrByte(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
	 
	 return status;
 }
 


 VL53L1X_ERROR VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t *pInterruptPolarity){
	 uint8_t Temp;
	 VL53L1X_ERROR status = 0;
 
	 status = VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
	 Temp = Temp & 0x10;
	 *pInterruptPolarity = !(Temp>>4);
	 
	 //ESP_LOGD(TAG, "Interrupt polarity for device 0x%02X: %d", dev, *pInterruptPolarity);
	 
	 return status;
 }
 


 VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev){
    VL53L1X_ERROR status = 0;
    
    ESP_LOGI(TAG, "StartRanging: Invio comando I2C a 0x%02X", dev);
    
    status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);	/* Enable VL53L1X */

    ESP_LOGI(TAG, "StartRanging: Stato scrittura = %d", status);
    
    if (status != 0) {
        ESP_LOGE(TAG, "Errore: Scrittura fallita su SYSTEM__MODE_START");
    }

    return status;
}

 

 VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev){
	 VL53L1X_ERROR status = 0;
	 //ESP_LOGI(TAG, "Stopping ranging for device 0x%02X", dev);
	 status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00);	/* Disable VL53L1X */
	 return status;
 }
 



 VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady) {
    uint8_t Temp;
    uint8_t IntPol;
    VL53L1X_ERROR status = 0;

    status = VL53L1X_GetInterruptPolarity(dev, &IntPol);
    if (status != 0) {
        ESP_LOGE(TAG, "Errore nel leggere Interrupt Polarity: %d", status);
        return status;
    }

    status = VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
    if (status != 0) {
        ESP_LOGE(TAG, "Errore nel leggere GPIO__TIO_HV_STATUS: %d", status);
        return status;
    }

    //ESP_LOGI(TAG, "GPIO__TIO_HV_STATUS: 0x%02X, IntPol: %d", Temp, IntPol);

    /* Read in the register to check if a new value is available */
    if ((Temp & 1) == IntPol) {
        *isDataReady = 1;
    } else {
        *isDataReady = 0;
    }

    //ESP_LOGI(TAG, "Data ready status: %d", *isDataReady);

    return status;
}

 


 VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs){
	uint16_t DM;
	VL53L1X_ERROR status = 0;
	 
	 //ESP_LOGI(TAG, "Setting timing budget for device 0x%02X to %u ms", dev, TimingBudgetInMs);
 
	 status = VL53L1X_GetDistanceMode(dev, &DM);
	 if (DM == 0) {
		 ESP_LOGE(TAG, "Invalid distance mode (0)");
		 return 1;
	 }
	 else if (DM == 1) {	/* Short DistanceMode */
		 switch (TimingBudgetInMs) {
		 case 15: /* only available in short distance mode */
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x01D);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x0027);
			 break;
		 case 20:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x0051);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x006E);
			 break;
		 case 33:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x00D6);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x006E);
			 break;
		 case 50:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x1AE);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x01E8);
			 break;
		 case 100:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x02E1);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x0388);
			 break;
		 case 200:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x03E1);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x0496);
			 break;
		 case 500:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x0591);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x05C1);
			 break;
		 default:
			 ESP_LOGE(TAG, "Invalid timing budget: %u ms", TimingBudgetInMs);
			 status = 1;
			 break;
		 }
	 } else {
		 switch (TimingBudgetInMs) {
		 case 20:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x001E);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x0022);
			 break;
		 case 33:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x0060);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x006E);
			 break;
		 case 50:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x00AD);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x00C6);
			 break;
		 case 100:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x01CC);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x01EA);
			 break;
		 case 200:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x02D9);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x02F8);
			 break;
		 case 500:
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,0x048F);
			 VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,0x04A4);
			 break;
		 default:
			 ESP_LOGE(TAG, "Invalid timing budget: %u ms", TimingBudgetInMs);
			 status = 1;
			 break;
		 }
	 }
	 
	 if (status == 0) {
		 ESP_LOGI(TAG, "Timing budget set successfully");
	 }
	 
	 return status;
 }
 


 


 VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t *pTimingBudget) {
    uint16_t Temp;
    VL53L1X_ERROR status = VL53L1_ERROR_NONE;

    // 📌 Controlla se il sensore è pronto leggendo SYSTEM_STATUS (0xE5)
    uint8_t system_status;
    status = VL53L1_RdByte(dev, 0xE5, &system_status);
    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore lettura SYSTEM_STATUS (0xE5), status = %d", status);
        return status;
    }
    ESP_LOGI(TAG, "VL53L1X SYSTEM_STATUS: 0x%02X", system_status);
    
    if (system_status != 0x03) {
        ESP_LOGE(TAG, "VL53L1X non è pronto! SYSTEM_STATUS = 0x%02X", system_status);
        return VL53L1_ERROR_CONTROL_INTERFACE;  // oppure VL53L1_ERROR_INVALID_PARAMS se più appropriato

    }

    // 📌 Lettura con retry del registro RANGE_CONFIG__TIMEOUT_MACROP_A_HI (0x5E)
    int retry_count = 5;
    while (retry_count--) {
        status = VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
        if (status == VL53L1_ERROR_NONE) {
            break;  // Lettura riuscita
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Attendi 10ms prima di riprovare
    }

    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Errore lettura registro 0x5E dopo più tentativi, status = %d", status);
        return status;
    }

    // 📌 Mappa il valore letto a un timing budget
    switch (Temp) {
        case 0x001D: *pTimingBudget = 15; break;
        case 0x0051:
        case 0x001E: *pTimingBudget = 20; break;
        case 0x00D6:
        case 0x0060: *pTimingBudget = 33; break;
        case 0x1AE:
        case 0x00AD: *pTimingBudget = 50; break;
        case 0x02E1:
        case 0x01CC: *pTimingBudget = 100; break;
        case 0x03E1:
        case 0x02D9: *pTimingBudget = 200; break;
        case 0x0591:
        case 0x048F: *pTimingBudget = 500; break;
        default:
            ESP_LOGE(TAG, "Valore sconosciuto nel registro 0x5E: 0x%04X", Temp);
            *pTimingBudget = 0;
            return VL53L1_ERROR_INVALID_PARAMS;
    }

    ESP_LOGI(TAG, "Timing budget per device 0x%02X: %u ms", dev, *pTimingBudget);
    return status;
}




 



 

VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DistanceMode) {
    ESP_LOGI(TAG, "Setting distance mode for device 0x%02X to %d", dev, DistanceMode);
    
    // Verify the system status first
    uint8_t system_status = 0;
    if (VL53L1X_SystemStatus(dev, &system_status) != 0 || system_status != 0x03) {
        ESP_LOGE(TAG, "System status not ready: 0x%02X", system_status);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    
    // Explicitly start I2C transaction for reading timing budget
    uint16_t tbValue = 0;
    //uint8_t phasecalValue = 0;
    
    // Start explicit transaction to read timing budget
    if (!i2c_start(dev, I2C_WRITE)) {
        ESP_LOGE(TAG, "Failed to start I2C transaction for reading timing budget");
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    
    // Send register address for timing budget (0x5E)
    uint8_t reg_addr[2] = {0x00, 0x5E};  // Register address in MSB, LSB format
    if (i2c_write(reg_addr, 2) != 2) {
        ESP_LOGE(TAG, "Failed to write register address for timing budget");
        i2c_transmit();  // End the transaction
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    
    // Complete write transaction
    i2c_transmit();
    
    // Small delay before starting read transaction
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Start read transaction
    if (!i2c_start(dev, I2C_READ)) {
        ESP_LOGE(TAG, "Failed to start I2C read transaction for timing budget");
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    
    // Read data
    uint8_t buffer[2];
    if (i2c_read(buffer, 2) != 2) {
        ESP_LOGE(TAG, "Failed to read timing budget data");
        i2c_transmit();  // End the transaction
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    
    // Complete read transaction
    i2c_transmit();
    
    tbValue = (buffer[0] << 8) | buffer[1];
    ESP_LOGI(TAG, "Current timing budget: %d", tbValue);
    
    // Rest of the function implementation...
    // Based on distance mode, set appropriate values
    
    if (DistanceMode == 1) {
        // Short distance mode
        
        // Explicitly write PHASECAL_CONFIG__TIMEOUT_MACROP
        if (!i2c_start(dev, I2C_WRITE)) {
            ESP_LOGE(TAG, "Failed to start I2C transaction for PHASECAL_CONFIG");
            return VL53L1_ERROR_CONTROL_INTERFACE;
        }
        
        // Register 0x5E with value 0x0D
        uint8_t phasecal_data[3] = {0x00, 0x5E, 0x0D};
        if (i2c_write(phasecal_data, 3) != 3) {
            ESP_LOGE(TAG, "Failed to write PHASECAL_CONFIG");
            i2c_transmit();
            return VL53L1_ERROR_CONTROL_INTERFACE;
        }
        
        i2c_transmit();
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Additional settings for short mode...
    } else {
        // Long distance mode
        
        // Explicitly write PHASECAL_CONFIG__TIMEOUT_MACROP
        if (!i2c_start(dev, I2C_WRITE)) {
            ESP_LOGE(TAG, "Failed to start I2C transaction for PHASECAL_CONFIG");
            return VL53L1_ERROR_CONTROL_INTERFACE;
        }
        
        // Register 0x5E with value 0x14
        uint8_t phasecal_data[3] = {0x00, 0x5E, 0x14};
        if (i2c_write(phasecal_data, 3) != 3) {
            ESP_LOGE(TAG, "Failed to write PHASECAL_CONFIG");
            i2c_transmit();
            return VL53L1_ERROR_CONTROL_INTERFACE;
        }
        
        i2c_transmit();
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Additional settings for long mode...
    }
    
    ESP_LOGI(TAG, "Distance mode successfully set to %d", DistanceMode);
    return VL53L1_ERROR_NONE;
}



 


VL53L1X_ERROR VL53L1X_GetDistanceMode(uint16_t dev, uint16_t *DM) {
    VL53L1X_ERROR status = 0;
    uint8_t vcsel_period;

    // Legge il registro RANGE_CONFIG__VCSEL_PERIOD_A
    status = VL53L1_RdByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, &vcsel_period);
    if (status != 0) {
        ESP_LOGE(TAG, "Errore lettura RANGE_CONFIG__VCSEL_PERIOD_A, status = %d", status);
        return status;
    }

    ESP_LOGI(TAG, "Valore RANGE_CONFIG__VCSEL_PERIOD_A letto: 0x%02X", vcsel_period);

    // Decodifica la modalità distanza
    if (vcsel_period == 0x07) {
        *DM = 1;  // Modalità corta
    } else if (vcsel_period == 0x0F) {
        *DM = 2;  // Modalità lunga
    } else {
        ESP_LOGW(TAG, "Valore RANGE_CONFIG__VCSEL_PERIOD_A sconosciuto: 0x%02X", vcsel_period);
        return 1;
    }

    ESP_LOGI(TAG, "Distance mode letto: %u", *DM);
    return 0;
}



VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t InterMeasMs){
	uint16_t ClockPLL;
	VL53L1X_ERROR status = 0;
	
	ESP_LOGI(TAG, "Setting inter-measurement period for device 0x%02X to %"PRIu32" ms", dev, InterMeasMs);

	status = VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL&0x3FF;
	
	uint32_t clockPeriod = (uint32_t)(ClockPLL * InterMeasMs * 1.075);
	ESP_LOGD(TAG, "Calculated clock period: %"PRIu32" (ClockPLL: %u)", clockPeriod, ClockPLL);
	
	status = VL53L1_WrDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, clockPeriod);
	
	if (status == 0) {
	    ESP_LOGI(TAG, "Inter-measurement period set successfully");
	} else {
	    ESP_LOGE(TAG, "Failed to set inter-measurement period");
	}
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t *pIM){
	uint16_t ClockPLL;
	VL53L1X_ERROR status = 0;
	uint32_t tmp;

	status = VL53L1_RdDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
	*pIM = (uint16_t)tmp;
	status = VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL&0x3FF;
	*pIM= (uint16_t)(*pIM/(ClockPLL*1.065));

	//ESP_LOGI(TAG, "Device 0x%02X inter-measurement period: %u ms", dev, *pIM);
	
	return status;
}


VL53L1X_ERROR VL53L1X_BootState(uint16_t dev, uint8_t *state){
	VL53L1X_ERROR status = 0;
	uint8_t tmp = 0;
	status = VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
	*state = tmp;
	//ESP_LOGI(TAG, "Device 0x%02X boot state: 0x%02X", dev, *state);
	return status;
}


VL53L1X_ERROR VL53L1X_GetSensorId(uint16_t dev, uint16_t *sensorId){
	VL53L1X_ERROR status = 0;
	uint16_t tmp = 0;
	status = VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
	*sensorId = tmp;
	//ESP_LOGI(TAG, "Device 0x%02X sensor ID: 0x%04X", dev, *sensorId);
	return status;
}


VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = (VL53L1_RdWord(dev,
			VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	
	//ESP_LOGD(TAG, "Device 0x%02X distance: %u mm", dev, *distance);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t *signalRate){
	VL53L1X_ERROR status = 0;
	uint16_t SpNb=1, signal;

	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
	*signalRate = (uint16_t) (200.0*signal/SpNb);
	
	//ESP_LOGD(TAG, "Device 0x%02X signal per SPAD: %u (signal: %u, SPADs: %u)", dev, *signalRate, signal, SpNb);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t *ambPerSp){
	VL53L1X_ERROR status = 0;
	uint16_t AmbientRate, SpNb = 1;

	status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
	status = VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
	*ambPerSp=(uint16_t) (200.0 * AmbientRate / SpNb);
	
	//ESP_LOGD(TAG, "Device 0x%02X ambient per SPAD: %u (ambient: %u, SPADs: %u)", dev, *ambPerSp, AmbientRate, SpNb);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetSignalRate(uint16_t dev, uint16_t *signal){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
	*signal = tmp*8;
	
	//ESP_LOGD(TAG, "Device 0x%02X signal rate: %u", dev, *signal);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetSpadNb(uint16_t dev, uint16_t *spNb){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev,VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);*spNb = tmp >> 8;
	
	//ESP_LOGD(TAG, "Device 0x%02X SPAD count: %u", dev, *spNb);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetAmbientRate(uint16_t dev, uint16_t *ambRate){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
	*ambRate = tmp*8;
	
	ESP_LOGD(TAG, "Device 0x%02X ambient rate: %u", dev, *ambRate);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetRangeStatus(uint16_t dev, uint8_t *rangeStatus){
	VL53L1X_ERROR status = 0;
	uint8_t RgSt;

	*rangeStatus = 255;
	status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	RgSt = RgSt & 0x1F;
	if (RgSt < 24)
		*rangeStatus = status_rtn[RgSt];
		
	//ESP_LOGI(TAG, "Device 0x%02X range status: %u (raw: 0x%02X)", dev, *rangeStatus, RgSt);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult){
	VL53L1X_ERROR status = 0;
	uint8_t Temp[17];
	uint8_t RgSt = 255;

	status = VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, Temp, 17);
	RgSt = Temp[0] & 0x1F;
	if (RgSt < 24)
		RgSt = status_rtn[RgSt];
	pResult->Status = RgSt;
	pResult->Ambient = (Temp[7] << 8 | Temp[8]) * 8;
	pResult->NumSPADs = Temp[3];
	pResult->SigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
	pResult->Distance = Temp[13] << 8 | Temp[14];
	
	//ESP_LOGI(TAG, "Device 0x%02X measurement result: Status=%u, Distance=%u mm, Ambient=%u, SPADs=%u, Signal/SPAD=%u", dev, pResult->Status, pResult->Distance, pResult->Ambient, pResult->NumSPADs, pResult->SigPerSPAD);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetOffset(uint16_t dev, int16_t OffsetValue){
	VL53L1X_ERROR status = 0;
	int16_t Temp;
	
	//ESP_LOGI(TAG, "Setting offset for device 0x%02X to %d mm", dev, OffsetValue);

	Temp = (OffsetValue*4);

	VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
			(uint16_t)Temp);
	VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	
	ESP_LOGI(TAG, "Offset set successfully (scaled value: %d)", Temp);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetOffset(uint16_t dev, int16_t *offset){
	VL53L1X_ERROR status = 0;
	uint16_t Temp;

	status = VL53L1_RdWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
	Temp = Temp<<3;
	Temp = Temp>>5;
	*offset = (int16_t)(Temp);
	
	//ESP_LOGI(TAG, "Device 0x%02X offset: %d mm", dev, *offset);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetXtalk(uint16_t dev, uint16_t XtalkValue){
/* XTalkValue in count per second to avoid float type */
	VL53L1X_ERROR status = 0;
	
	ESP_LOGI(TAG, "Setting crosstalk for device 0x%02X to %u cps", dev, XtalkValue);

	status = VL53L1_WrWord(dev,ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,0x0000);
	status = VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,0x0000);
			
	uint16_t xtalkReg = (XtalkValue<<9)/1000; /* << 9 (7.9 format) and /1000 to convert cps to kpcs */

	//ESP_LOGD(TAG, "Calculated crosstalk register value: %u", xtalkReg);
	
	status = VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalkReg);
	
	//ESP_LOGI(TAG, "Crosstalk value set successfully");
	
	return status;
}



VL53L1X_ERROR VL53L1X_GetXtalk(uint16_t dev, uint16_t *xtalk){
	VL53L1X_ERROR status = 0;

	status = VL53L1_RdWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalk);
	*xtalk = (uint16_t)((*xtalk*1000)>>9); /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
	
	//ESP_LOGI(TAG, "Device 0x%02X crosstalk: %u cps", dev, *xtalk);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t ThreshLow, uint16_t ThreshHigh, uint8_t Window, uint8_t IntOnNoTarget){
	VL53L1X_ERROR status = 0;
	uint8_t Temp = 0;
	
	//ESP_LOGI(TAG, "Setting distance threshold for device 0x%02X: Low=%u mm, High=%u mm, Window=%u, IntOnNoTarget=%u",dev, ThreshLow, ThreshHigh, Window, IntOnNoTarget);

	status = VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
	Temp = Temp & 0x47;
	if (IntOnNoTarget == 0) {
		status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (Temp | (Window & 0x07)));
	} else {
		status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, ((Temp | (Window & 0x07)) | 0x40));
	}
	status = VL53L1_WrWord(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
	status = VL53L1_WrWord(dev, SYSTEM__THRESH_LOW, ThreshLow);
	
	//ESP_LOGI(TAG, "Distance threshold set successfully");
	
	return status;
}



VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t *window){
	VL53L1X_ERROR status = 0;
	uint8_t tmp;
	
	status = VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
	*window = (uint16_t)(tmp & 0x7);
	
	//ESP_LOGI(TAG, "Device 0x%02X threshold window: %u", dev, *window);
	
	return status;
}



VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev, SYSTEM__THRESH_LOW, &tmp);
	*low = tmp;
	
	//ESP_LOGI(TAG, "Device 0x%02X low threshold: %u mm", dev, *low);
	
	return status;
}



VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev, SYSTEM__THRESH_HIGH, &tmp);
	*high = tmp;
	
	//ESP_LOGI(TAG, "Device 0x%02X high threshold: %u mm", dev, *high);
	
	return status;
}



VL53L1X_ERROR VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter){
	VL53L1X_ERROR status = 0;
	
	//ESP_LOGI(TAG, "Setting ROI center for device 0x%02X to 0x%02X", dev, ROICenter);
	
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
	
	return status;
}



VL53L1X_ERROR VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter){
	VL53L1X_ERROR status = 0;
	uint8_t tmp;
	
	status = VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, &tmp);
	*ROICenter = tmp;
	
	//ESP_LOGI(TAG, "Device 0x%02X ROI center: 0x%02X", dev, *ROICenter);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y){
	uint8_t OpticalCenter;
	VL53L1X_ERROR status = 0;
	
	//ESP_LOGI(TAG, "Setting ROI for device 0x%02X: X=%u, Y=%u", dev, X, Y);

	status = VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10){
		OpticalCenter = 199;
	}
	
	//ESP_LOGD(TAG, "Optical center: 0x%02X", OpticalCenter);
	
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,(Y - 1) << 4 | (X - 1));
		       
	//ESP_LOGI(TAG, "ROI set successfully");
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y){
	VL53L1X_ERROR status = 0;
	uint8_t tmp;

	status = VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
	*ROI_X = ((uint16_t)tmp & 0x0F) + 1;
	*ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
	
	//ESP_LOGI(TAG, "Device 0x%02X ROI: X=%u, Y=%u", dev, *ROI_X, *ROI_Y);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t Signal){
	VL53L1X_ERROR status = 0;
	
	//ESP_LOGI(TAG, "Setting signal threshold for device 0x%02X to %u", dev, Signal);

	status = VL53L1_WrWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, Signal>>3);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t *signal){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
	*signal = tmp <<3;
	
	//ESP_LOGI(TAG, "Device 0x%02X signal threshold: %u", dev, *signal);
	
	return status;
}


VL53L1X_ERROR VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t Sigma){
	VL53L1X_ERROR status = 0;
	
	ESP_LOGI(TAG, "Setting sigma threshold for device 0x%02X to %u", dev, Sigma);

	if(Sigma > (0xFFFF>>2)){
	    //ESP_LOGE(TAG, "Sigma value too high: %u (max: %u)", Sigma, 0xFFFF>>2);
		return 1;
	}
	/* 16 bits register 14.2 format */
	status = VL53L1_WrWord(dev, RANGE_CONFIG__SIGMA_THRESH, Sigma<<2);
	
	return status;
}


VL53L1X_ERROR VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t *sigma){
	VL53L1X_ERROR status = 0;
	uint16_t tmp;

	status = VL53L1_RdWord(dev, RANGE_CONFIG__SIGMA_THRESH, &tmp);
	*sigma = tmp >> 2;
	
	//ESP_LOGI(TAG, "Device 0x%02X sigma threshold: %u", dev, *sigma);
	
	return status;
}



VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(uint16_t dev){
	VL53L1X_ERROR status = 0;
	uint8_t tmp=0;
	
	//ESP_LOGI(TAG, "Starting temperature update for device 0x%02X", dev);

	status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81); /* full VHV */
	status = VL53L1_WrByte(dev, 0x0B, 0x92);
	
	//ESP_LOGI(TAG, "Starting ranging for temperature calibration");

	status = VL53L1X_StartRanging(dev);
	
	while(tmp == 0){
		status = VL53L1X_CheckForDataReady(dev, &tmp);
	}
	
	//ESP_LOGI(TAG, "Temperature calibration measurement complete");
	
	tmp = 0;
	status = VL53L1X_ClearInterrupt(dev);
	status = VL53L1X_StopRanging(dev);
	
	status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	status = VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
	
	//ESP_LOGI(TAG, "Temperature update completed successfully");
	
	return status;
}