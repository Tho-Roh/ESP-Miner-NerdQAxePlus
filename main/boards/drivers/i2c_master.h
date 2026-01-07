#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_NUM ((i2c_port_t)0)
#define I2C_MASTER_TIMEOUT_TICKS pdMS_TO_TICKS(1000)

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_delete(void);

// -----------------------------------------------------------------------------
// Port-aware low level API
// -----------------------------------------------------------------------------

esp_err_t i2c_master_register_read_port(i2c_port_t port,
                                        uint8_t device_address,
                                        uint8_t reg_addr,
                                        uint8_t *data,
                                        size_t len);

esp_err_t i2c_master_register_write_port(i2c_port_t port,
                                         uint8_t device_address,
                                         uint8_t reg_addr,
                                         const uint8_t *data,
                                         size_t len);

// -----------------------------------------------------------------------------
// Legacy / convenience wrappers (project standard)
// -----------------------------------------------------------------------------

static inline esp_err_t i2c_master_read_reg(i2c_port_t port,
                                            uint8_t addr,
                                            uint8_t reg,
                                            uint8_t* data,
                                            size_t len)
{
    // FIX: port parameter is now respected
    return i2c_master_register_read_port(port, addr, reg, data, len);
}

static inline esp_err_t i2c_master_write_reg(i2c_port_t port,
                                             uint8_t addr,
                                             uint8_t reg,
                                             uint8_t* data,
                                             size_t len)
{
    return i2c_master_register_write_port(port, addr, reg, data, len);
}

// -----------------------------------------------------------------------------
// Backward compatibility helpers (used by older drivers)
// -----------------------------------------------------------------------------

static inline esp_err_t i2c_master_register_read(uint8_t device_address,
                                                 uint8_t reg_addr,
                                                 uint8_t *data,
                                                 size_t len)
{
    return i2c_master_register_read_port(
        I2C_MASTER_NUM, device_address, reg_addr, data, len);
}

static inline esp_err_t i2c_master_register_write_byte(uint8_t device_address,
                                                       uint8_t reg_addr,
                                                       uint8_t data)
{
    return i2c_master_register_write_port(
        I2C_MASTER_NUM, device_address, reg_addr, &data, 1);
}
