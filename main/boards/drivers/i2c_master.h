#pragma once
#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_NUM ((i2c_port_t) 0)
#define I2C_MASTER_TIMEOUT_TICKS pdMS_TO_TICKS(1000)

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_delete(void);

// Standard-API für das Projekt
esp_err_t i2c_master_register_read(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_master_register_write_byte(uint8_t device_address, uint8_t reg_addr, uint8_t data);
esp_err_t i2c_master_register_write_word(uint8_t device_address, uint8_t reg_addr, uint16_t data);

// NEU: Kompatibilitäts-Wrapper für TMP468/451 Treiber (2026 Standard)
static inline esp_err_t i2c_master_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_register_read(addr, reg, data, len);
}
static inline esp_err_t i2c_master_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    if (len == 1) return i2c_master_register_write_byte(addr, reg, *data);
    if (len == 2) return i2c_master_register_write_word(addr, reg, (data[0] << 8) | data[1]);
    return ESP_ERR_NOT_SUPPORTED;
}
