#include "TMP1075.h"
#include "i2c_master.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "TMP1075";

// -----------------------------------------------------------------------------
// ESP-IDF native register access
// -----------------------------------------------------------------------------

static esp_err_t TMP1075_read_word(uint8_t addr,
                                  uint8_t reg,
                                  uint16_t *out)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_register_read(addr, reg, buf, 2);
    if (err != ESP_OK)
        return err;

    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

float TMP1075_read_temperature(int device_index)
{
    uint16_t raw;

    if (TMP1075_read_word(TMP1075_I2CADDR_DEFAULT + device_index,
                          TMP1075_TEMP_REG,
                          &raw) != ESP_OK) {
        ESP_LOGE(TAG, "TMP1075 read failed");
        return NAN;
    }

    // TMP1075: signed 12-bit, LSB = 0.0625°C
    int16_t temp = raw >> 4;
    return temp * 0.0625f;
}
