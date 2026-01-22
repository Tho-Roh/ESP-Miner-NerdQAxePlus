#include "TMP468.h"
#include <esp_check.h>
#include <math.h>

// Software calibration (same model as TMP451)
struct TempCal {
    float scale;
    float off[8];
};

static TempCal gCal = {
    1.09f,
    { -29.5f, -29.5f, -29.5f, -29.5f,
      -29.5f, -29.5f, -29.5f, -29.5f }
};

TMP468::TMP468(uint8_t addr, uint8_t asicCount)
    : m_addr(addr), m_asicCount(asicCount) {}

esp_err_t TMP468::read_reg(uint8_t reg, uint8_t* out)
{
    return i2c_master_register_read(m_addr, reg, out, 1);
}

esp_err_t TMP468::write_reg(uint8_t reg, uint8_t val)
{
    return i2c_master_register_write_byte(m_addr, reg, val);
}

esp_err_t TMP468::read_reg_16(uint8_t reg, uint8_t* msb, uint8_t* lsb)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_register_read(m_addr, reg, buf, 2);
    if (err != ESP_OK) return err;
    *msb = buf[0];
    *lsb = buf[1];
    return ESP_OK;
}

esp_err_t TMP468::init()
{
    uint8_t msb, lsb;

    // Check manufacturer ID (expected 0x5449 per datasheet)
    ESP_RETURN_ON_ERROR(
        read_reg_16(TMP468_REG_MAN_ID, &msb, &lsb),
        TAG, "Manufacturer ID read failed"
    );

    uint16_t mid = (msb << 8) | lsb;
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_NOT_FOUND;
    }

    // Device ID (optional read/log)
    uint8_t devId;
    if (read_reg(TMP468_REG_DEVICE_ID, &devId) == ESP_OK) {
        ESP_LOGI(TAG, "TMP468 Device ID: 0x%02X", devId);
    }

    // Perform software reset to ensure POR state if supported
    // Software reset register = 0x20 (bit15) => set high byte 0x80, low byte 0x00
    uint8_t rst_hi = 0x80, rst_lo = 0x00;
    i2c_master_register_write_word(m_addr, 0x20, ((uint16_t)rst_hi<<8) | rst_lo);

    vTaskDelay(pdMS_TO_TICKS(5)); // short delay after reset

    // Default config: enable continuous conversion
    ESP_RETURN_ON_ERROR(
        write_reg(TMP468_REG_CONFIG, TMP468_CONFIG_POR),
        TAG, "Config write failed"
    );

    // Clear N-Factor and Offset for all channels
    for (uint8_t ch = 1; ch <= 8; ch++) {
        write_reg(TMP468_OFFSET_REG(ch), 0x00);
        write_reg(TMP468_NFACTOR_REG(ch), 0x00);
    }

    // Optional: unlock registers if you plan to change limits/config
    // write unlock word to lock register (C4h) if needed

    ESP_LOGI(TAG, "TMP468 initialized @0x%02X", m_addr);
    return ESP_OK;
}

bool TMP468::readStatus(uint8_t* out_status)
{
    if (!out_status) return false;
    return read_reg(TMP468_REG_STATUS, out_status) == ESP_OK;
}

float TMP468::read_local_celsius()
{
    uint8_t msb, lsb;
    // Temperature register 0x00 contains 13-bit signed
    if (read_reg_16(TMP468_REG_TEMP_BASE + 0, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

float TMP468::read_remote_celsius(uint8_t channel)
{
    if (channel < 1 || channel > 8) return NAN;
    uint8_t msb, lsb;
    if (read_reg_16(TMP468_REG_TEMP_BASE + channel, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

bool TMP468::readLocalTemp(float* out_C)
{
    if (!out_C) return false;
    float t = read_local_celsius();
    if (isnan(t)) return false;
    *out_C = t;
    return true;
}

float TMP468::get_temperature(int index)
{
    if (index == -1)
        return read_local_celsius();

    if (index < 0 || index >= m_asicCount)
        return NAN;

    uint8_t ch = index + 1;

    // no MUX switching – direct read
    float raw = read_remote_celsius(ch);

    // Correction: apply scale + offset
    return temp_correct(ch, raw);
}

float TMP468::temp_correct(uint8_t ch, float t)
{
    if (isnan(t) || ch < 1 || ch > 8) return t;
    return ((t - 30.0f) * gCal.scale + 30.0f) + gCal.off[ch - 1];
}
