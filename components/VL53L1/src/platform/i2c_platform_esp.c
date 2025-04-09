
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "i2c_platform_esp.h"
#include <inttypes.h>
#include "sdkconfig.h"


static const char *TAG = "I2C";



I2C_Master_t *I2C_Master = NULL;



void i2c_scan() {

    if (I2C_Master == NULL || I2C_Master->bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "I2C device scan starting...");
    bool devices_found = false;
    uint8_t device_count = 0;
    
    for (uint8_t i = 1; i < 127; i++) {
        esp_err_t ret;
        ret = i2c_master_probe(I2C_Master->bus_handle, i, 100);
        
        if (ret == ESP_OK) {
            devices_found = true;
            device_count++;
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", i);
        }
    }
    
    if (!devices_found) {
        ESP_LOGW(TAG, "No I2C devices found on the bus");
    } else {
        ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", device_count);
    }
}


I2C_Master_t *i2c_master_setup() {
    I2C_Master_t *new_master = (I2C_Master_t *) malloc(sizeof(I2C_Master_t));
    if (new_master == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2C master");
        return NULL;
    }
    
    new_master->port = CONFIG_VL53L1_I2C_DEFAULT_PORT;
    new_master->pin_sda = CONFIG_VL53L1_I2C_DEFAULT_SDA;
    new_master->pin_scl = CONFIG_VL53L1_I2C_DEFAULT_SCL;
    new_master->freq = CONFIG_VL53L1_I2C_DEFAULT_FREQ;
    new_master->dev_address = I2C_NO_DEVICE;
    new_master->bus_handle = NULL;
    new_master->dev_handle = NULL;
    new_master->transaction_in_progress = false;
    new_master->last_register_addr = 0;
    return new_master;
}

void i2c_get_config(i2c_port_t *port, gpio_num_t *pin_sda, gpio_num_t *pin_scl, uint32_t *freq) {
    if (I2C_Master == NULL) {
        ESP_LOGE(TAG, "I2C not initialized");
        return;
    }
    
    *port = I2C_Master->port;
    *pin_sda = I2C_Master->pin_sda;
    *pin_scl = I2C_Master->pin_scl;
    *freq = I2C_Master->freq;
}

void i2c_init_driver() {
    if (I2C_Master == NULL) {
        ESP_LOGE(TAG, "I2C master not initialized");
        return;
    }
    
    // Create a new I2C master bus configuration
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_Master->port,
        .sda_io_num = I2C_Master->pin_sda,
        .scl_io_num = I2C_Master->pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Initialize the I2C master bus
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &I2C_Master->bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(err));
    }
}

void i2c_init_config(i2c_port_t port, gpio_num_t pin_sda, gpio_num_t pin_scl, uint32_t freq) {
    if (I2C_Master != (I2C_Master_t *) NULL) return;
    
    I2C_Master = i2c_master_setup();
    if (I2C_Master == NULL) return;
    
    I2C_Master->port = port;
    I2C_Master->pin_sda = pin_sda;
    I2C_Master->pin_scl = pin_scl;
    I2C_Master->freq = freq;
    i2c_init_driver();
    
    // Piccolo ritardo dopo l'inizializzazione
    vTaskDelay(pdMS_TO_TICKS(10));
}

void i2c_init() {
    if (I2C_Master != (I2C_Master_t *) NULL) return;
    
    I2C_Master = i2c_master_setup();
    if (I2C_Master == NULL) return;
    
    i2c_init_driver();
    
    // Piccolo ritardo dopo l'inizializzazione
    vTaskDelay(pdMS_TO_TICKS(10));
}

void i2c_remove() {
    if (I2C_Master == NULL) return;
    
    if (I2C_Master->dev_handle != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(I2C_Master->dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(err));
        }
        I2C_Master->dev_handle = NULL;
    }
    
    if (I2C_Master->bus_handle != NULL) {
        esp_err_t err = i2c_del_master_bus(I2C_Master->bus_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete master bus: %s", esp_err_to_name(err));
        }
        I2C_Master->bus_handle = NULL;
    }
    
    free(I2C_Master);
    I2C_Master = (I2C_Master_t *) NULL;
}

void i2c_upgrade(uint32_t upgrade_freq) {
    if (I2C_Master == NULL) {
        ESP_LOGE(TAG, "I2C not initialized");
        return;
    }
    
    i2c_port_t port;
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    uint32_t old_freq;
    
    i2c_get_config(&port, &pin_sda, &pin_scl, &old_freq);
    i2c_remove();
    i2c_init_config(port, pin_sda, pin_scl, upgrade_freq);
}

bool i2c_start(uint8_t i2c_device_address, uint8_t read_write) {
    if (I2C_Master == NULL || I2C_Master->bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }
    
    // Se c'è già una transazione in corso, terminiamola prima
    if (I2C_Master->transaction_in_progress) {
        i2c_transmit();
    }
    
    // If there's already a device handle with a different address, remove it
    if (I2C_Master->dev_handle != NULL && I2C_Master->dev_address != i2c_device_address) {
        esp_err_t err = i2c_master_bus_rm_device(I2C_Master->dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(err));
            return false;
        }
        I2C_Master->dev_handle = NULL;
    }
    
    // If no device handle or different address, create a new one
    if (I2C_Master->dev_handle == NULL) {
        // Configure the I2C device
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i2c_device_address,
            .scl_speed_hz = I2C_Master->freq,
        };
        
        // Add device to the bus
        esp_err_t err = i2c_master_bus_add_device(I2C_Master->bus_handle, &dev_cfg, &I2C_Master->dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(err));
            return false;
        }
        I2C_Master->dev_address = i2c_device_address;
        
        // Piccolo ritardo dopo aver aggiunto il dispositivo
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Mark that we're starting a transaction
    I2C_Master->transaction_in_progress = true;
    
    return true;
}




size_t i2c_write_byte(uint8_t data_byte_out) {

    if (I2C_Master == NULL || I2C_Master->dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return 0;
    }

    // Prepara il messaggio da inviare
    uint8_t write_buffer = data_byte_out;
    
    // Log dettagliato prima di scrivere
    ESP_LOGI(TAG, "Writing byte 0x%02X to device 0x%02X", write_buffer, I2C_Master->dev_address);

    // Verifica che il dispositivo risponda prima di scrivere
    esp_err_t probe_ret = i2c_master_probe(I2C_Master->bus_handle, I2C_Master->dev_address, 100);
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG, "Device 0x%02X not responding before write! Error: %s", 
                 I2C_Master->dev_address, esp_err_to_name(probe_ret));
        return 0;
    }

    // Salva il byte inviato se potrebbe essere un indirizzo di registro
    I2C_Master->last_register_addr = write_buffer;

    // Scrive il byte utilizzando la nuova API
    esp_err_t err = i2c_master_transmit(I2C_Master->dev_handle, &write_buffer, 1, -1);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write byte 0x%02X to device 0x%02X: %s", 
                 write_buffer, I2C_Master->dev_address, esp_err_to_name(err));

        // Controlla se il bus è bloccato
        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Resetting I2C bus due to error: %s", esp_err_to_name(err));
            i2c_master_bus_reset(I2C_Master->bus_handle);
            vTaskDelay(pdMS_TO_TICKS(10));  // Breve attesa dopo il reset

            // Riprova dopo il reset
            err = i2c_master_transmit(I2C_Master->dev_handle, &write_buffer, 1, -1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed again after reset: %s", esp_err_to_name(err));
                return 0;
            }
        } else {
            return 0;
        }
    }

    ESP_LOGI(TAG, "Successfully wrote byte 0x%02X to device 0x%02X", write_buffer, I2C_Master->dev_address);

    return 1;
}




size_t i2c_write(uint8_t *pByteBuffer, size_t NumByteToWrite) {
    if (I2C_Master == NULL || I2C_Master->dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return 0;
    }
    
    if (!I2C_Master->transaction_in_progress) {
        ESP_LOGW(TAG, "I2C transaction not started, auto-starting with last device");
        if (!i2c_start(I2C_Master->dev_address, I2C_WRITE)) {
            ESP_LOGE(TAG, "Failed to auto-start transaction");
            return 0;
        }
    }
    
    if (pByteBuffer == NULL || NumByteToWrite == 0) {
        ESP_LOGE(TAG, "Invalid buffer or size");
        return 0;
    }
    
    // Se stiamo inviando un solo byte, potrebbe essere un indirizzo di registro
    if (NumByteToWrite == 1) {
        I2C_Master->last_register_addr = pByteBuffer[0];
    }
    
    esp_err_t err = i2c_master_transmit(I2C_Master->dev_handle, pByteBuffer, NumByteToWrite, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write buffer: %s", esp_err_to_name(err));
        
        // Tenta di ripristinare il bus in caso di errore
        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Resetting I2C bus due to error");
            i2c_master_bus_reset(I2C_Master->bus_handle);
            
            // Piccolo ritardo dopo il reset
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Riprova dopo il reset
            err = i2c_master_transmit(I2C_Master->dev_handle, pByteBuffer, NumByteToWrite, -1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed again after reset: %s", esp_err_to_name(err));
                I2C_Master->transaction_in_progress = false;
                return 0;
            }
        } else {
            I2C_Master->transaction_in_progress = false;
            return 0;
        }
    }
    
    return NumByteToWrite;
}






int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    // First, start transaction and write register address
    if (!i2c_start(dev, I2C_WRITE)) {
        ESP_LOGE(TAG, "Failed to start I2C write transaction");
        return -1;
    }
    
    // Send register address (MSB first, then LSB)
    uint8_t reg_addr[2] = {(uint8_t)(index >> 8), (uint8_t)(index & 0xFF)};
    if (i2c_write(reg_addr, 2) != 2) {
        ESP_LOGE(TAG, "Failed to write register address");
        i2c_transmit(); // End transaction
        return -1;
    }
    
    // End write transaction
    i2c_transmit();
    
    // Add a small delay before starting read transaction
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Start read transaction
    if (!i2c_start(dev, I2C_READ)) {
        ESP_LOGE(TAG, "Failed to start I2C read transaction");
        return -1;
    }
    
    // Read data
    int read_count = i2c_read(pdata, count);
    
    // End read transaction
    i2c_transmit();
    
    if (read_count != count) {
        ESP_LOGE(TAG, "Failed to read %" PRIu32 " bytes, got %d", count, read_count);
        return -1;
    }
    
    return 0; // Success
}







uint8_t i2c_read_byte() {
    uint8_t byteBuffer = 0;
    
    if (I2C_Master == NULL || I2C_Master->dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return 0;
    }
    
    if (!I2C_Master->transaction_in_progress) {
        ESP_LOGW(TAG, "I2C transaction not started, auto-starting with last device");
        if (!i2c_start(I2C_Master->dev_address, I2C_READ)) {
            ESP_LOGE(TAG, "Failed to auto-start transaction");
            return 0;
        }
    }
    
    esp_err_t err;
    
    // Se abbiamo scritto un byte prima della lettura, potrebbe essere un indirizzo di registro
    // In questo caso, dobbiamo usare transmit-receive invece di receive
    if (I2C_Master->last_register_addr != 0) {
        uint8_t reg_addr = I2C_Master->last_register_addr;
        I2C_Master->last_register_addr = 0;  // Resetta per la prossima volta
        
        err = i2c_master_transmit_receive(I2C_Master->dev_handle, &reg_addr, 1, &byteBuffer, 1, -1);
    } else {
        err = i2c_master_receive(I2C_Master->dev_handle, &byteBuffer, 1, -1);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read byte: %s", esp_err_to_name(err));
        
        // Tenta di ripristinare il bus in caso di errore
        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Resetting I2C bus due to error");
            i2c_master_bus_reset(I2C_Master->bus_handle);
            
            // Piccolo ritardo dopo il reset
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Riprova dopo il reset, ma questa volta solo con receive
            err = i2c_master_receive(I2C_Master->dev_handle, &byteBuffer, 1, -1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed again after reset: %s", esp_err_to_name(err));
                I2C_Master->transaction_in_progress = false;
                return 0;
            }
        } else {
            I2C_Master->transaction_in_progress = false;
            return 0;
        }
    }
    
    return byteBuffer;
}



size_t i2c_read(uint8_t *pByteBuffer, size_t NumByteToRead) {
    if (I2C_Master == NULL || I2C_Master->dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return 0;
    }
    
    if (!I2C_Master->transaction_in_progress) {
        ESP_LOGW(TAG, "I2C transaction not started, auto-starting with last device");
        if (!i2c_start(I2C_Master->dev_address, I2C_READ)) {
            ESP_LOGE(TAG, "Failed to auto-start transaction");
            return 0;
        }
    }
    
    if (pByteBuffer == NULL || NumByteToRead == 0) {
        ESP_LOGE(TAG, "Invalid buffer or size");
        return 0;
    }
    
    ESP_LOGV(TAG, "Reading %d bytes from device 0x%02X", NumByteToRead, I2C_Master->dev_address);
    
    esp_err_t err;
    
    // If we have a last register address, use transmit-receive
    if (I2C_Master->last_register_addr != 0) {
        uint8_t reg_addr = I2C_Master->last_register_addr;
        ESP_LOGI(TAG, "Using last register address 0x%02X", reg_addr);
        I2C_Master->last_register_addr = 0;  // Reset for next time
        
        err = i2c_master_transmit_receive(I2C_Master->dev_handle, &reg_addr, 1, pByteBuffer, NumByteToRead, -1);
    } else {
        ESP_LOGV(TAG, "No register address, using direct receive");
        err = i2c_master_receive(I2C_Master->dev_handle, pByteBuffer, NumByteToRead, -1);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read buffer: %s", esp_err_to_name(err));
        
        // Attempt to recover the bus in case of error
        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Resetting I2C bus due to error");
            i2c_master_bus_reset(I2C_Master->bus_handle);
            
            // Small delay after reset
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Retry after reset, but only with receive
            err = i2c_master_receive(I2C_Master->dev_handle, pByteBuffer, NumByteToRead, -1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed again after reset: %s", esp_err_to_name(err));
                I2C_Master->transaction_in_progress = false;
                return 0;
            }
        } else {
            I2C_Master->transaction_in_progress = false;
            return 0;
        }
    }
    
    ESP_LOGV(TAG, "Successfully read %d bytes", NumByteToRead);
    
    return NumByteToRead;
}



esp_err_t i2c_transmit() {
    if (I2C_Master == NULL) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // In the new API, transmissions happen immediately in the write/read functions
    // We just mark the transaction as complete here
    I2C_Master->transaction_in_progress = false;
    I2C_Master->last_register_addr = 0;  // Reset last register address
    return ESP_OK;
}