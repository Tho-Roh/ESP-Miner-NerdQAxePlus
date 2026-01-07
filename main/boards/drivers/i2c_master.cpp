#include "i2c_master.h"

#define I2C_MASTER_SCL_IO 43
#define I2C_MASTER_SDA_IO 44
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}

esp_err_t i2c_master_delete(void)
{
    return i2c_driver_delete(I2C_MASTER_NUM);
}

// -----------------------------------------------------------------------------
// Port-aware register access
// -----------------------------------------------------------------------------

esp_err_t i2c_master_register_read_port(i2c_port_t port,
                                        uint8_t device_address,
                                        uint8_t reg_addr,
                                        uint8_t *data,
                                        size_t len)
{
    return i2c_master_write_read_device(
        port,
        device_address,
        &reg_addr,
        1,
        data,
        len,
        I2C_MASTER_TIMEOUT_TICKS);
}

esp_err_t i2c_master_register_write_port(i2c_port_t port,
                                         uint8_t device_address,
                                         uint8_t reg_addr,
                                         const uint8_t *data,
                                         size_t len)
{
    uint8_t buf[3];

    if (len > 2)
        return ESP_ERR_NOT_SUPPORTED;

    buf[0] = reg_addr;
    for (size_t i = 0; i < len; i++)
        buf[i + 1] = data[i];

    return i2c_master_write_to_device(
        port,
        device_address,
        buf,
        len + 1,
        I2C_MASTER_TIMEOUT_TICKS);
}
