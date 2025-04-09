/**
 * i2c_platform_esp.h
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html
 */

 #ifndef _I2C_PLATFORM_ESP_H_
 #define _I2C_PLATFORM_ESP_H_
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "driver/i2c_master.h"
 #include "driver/gpio.h"
 #include "esp_err.h"
 #include "esp_log.h"
 
 /**
  * @brief Default I2C configuration
  */
 //#define I2C_DEFAULT_PORT    (I2C_NUM_0)
 //#define I2C_DEFAULT_SDA     (GPIO_NUM_8)
 //#define I2C_DEFAULT_SCL     (GPIO_NUM_9)
 //#define I2C_DEFAULT_FREQ    (400000)
 
 /**
  * @brief I2C read/write bit values
  * Questi valori devono essere compatibili con quelli definiti in ESP-IDF
  */
 #define I2C_READ            (1)
 #define I2C_WRITE           (0)
 
 /**
  * @brief Special value to indicate no device is selected
  */
 #define I2C_NO_DEVICE       (0xFF)
 
 /**
  * @brief Enable acknowledgment check by default
  */
 #define ACK_CHECK_EN        true
 
 /**
  * @brief Convenience macros for starting I2C transactions
  */
 #define i2c_start_write(dev_address) i2c_start((dev_address), I2C_WRITE)
 #define i2c_start_read(dev_address) i2c_start((dev_address), I2C_READ)
 
 /**
  * @brief Struct per la gestione del master I2C
  */
 typedef struct {
     i2c_port_t        port;
     gpio_num_t        pin_sda;
     gpio_num_t        pin_scl;
     uint32_t          freq;
     uint8_t           dev_address;
     i2c_master_bus_handle_t bus_handle;
     i2c_master_dev_handle_t dev_handle;
     bool              transaction_in_progress;
     uint8_t           last_register_addr;
 } I2C_Master_t;
 
 /**
  * @brief Dichiarazione della variabile globale I2C_Master
  */
 extern I2C_Master_t *I2C_Master;
 
 /**
  * @brief Scan the I2C bus for connected devices
  */
 void i2c_scan();
 
 /**
  * @brief Get the current I2C configuration
  */
 void i2c_get_config(i2c_port_t *port, gpio_num_t *pin_sda, gpio_num_t *pin_scl, uint32_t *freq);
 
 /**
  * @brief Initialize I2C with specific configuration
  */
 void i2c_init_config(i2c_port_t port, gpio_num_t pin_sda, gpio_num_t pin_scl, uint32_t freq);
 
 /**
  * @brief Initialize I2C with default configuration
  */
 void i2c_init();
 
 /**
  * @brief Remove I2C driver and free resources
  */
 void i2c_remove();
 
 /**
  * @brief Upgrade I2C bus frequency
  */
 void i2c_upgrade(uint32_t upgrade_freq);
 
 /**
  * @brief Start an I2C transaction with a device
  */
 bool i2c_start(uint8_t i2c_device_address, uint8_t read_write);
 
 /**
  * @brief Write a single byte to the I2C device
  */
 size_t i2c_write_byte(uint8_t data_byte_out);
 
 /**
  * @brief Write multiple bytes to the I2C device
  */
 size_t i2c_write(uint8_t *pByteBuffer, size_t NumByteToWrite);
 


 int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);



 /**
  * @brief Read a single byte from the I2C device
  */
 uint8_t i2c_read_byte();
 
 /**
  * @brief Read multiple bytes from the I2C device
  */
 size_t i2c_read(uint8_t *pByteBuffer, size_t NumByteToRead);
 
 /**
  * @brief Complete I2C transaction
  */
 esp_err_t i2c_transmit();
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // _I2C_PLATFORM_ESP_H_