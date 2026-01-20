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

    // Manufacturer ID check (0x5449)
    ESP_RETURN_ON_ERROR(
        read_reg_16(TMP468_REG_MAN_ID, &msb, &lsb),
        TAG, "Manufacturer ID read failed"
    );

    uint16_t mid = (msb << 8) | lsb;
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_NOT_FOUND;
    }

    // Optional Device ID (logging only)
    uint8_t devId;
    if (read_reg(TMP468_REG_DEVICE_ID, &devId) == ESP_OK) {
        ESP_LOGI(TAG, "TMP468 Device ID: 0x%02X", devId);
    }

    // Datasheet POR config = 0x9C (continuous conversion)
    ESP_RETURN_ON_ERROR(
        write_reg(TMP468_REG_CONFIG, TMP468_CONFIG_POR),
        TAG, "Config write failed"
    );

    // Reset HW calibration registers
    for (uint8_t ch = 1; ch <= 8; ch++) {
        write_reg(TMP468_OFFSET_REG(ch), 0x00);
        write_reg(TMP468_NFACTOR_REG(ch), 0x00);
    }

    ESP_LOGI(TAG, "TMP468 initialized @0x%02X", m_addr);
    return ESP_OK;
}

bool TMP468::readStatus(uint8_t* out_status)
{
    if (!out_status) return false;
    // NOTE: Alarm / fault bits, NOT TMP451 compatible
    return read_reg(TMP468_REG_STATUS, out_status) == ESP_OK;
}

float TMP468::read_local_celsius()
{
    uint8_t msb, lsb;
    if (read_reg_16(TMP468_REG_TEMP_BASE, &msb, &lsb) == ESP_OK)
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
// -----------------------------------------------------------------------------
// ITempMux API
// -----------------------------------------------------------------------------

float TMP468::get_temperature(int index)
{
    if (index == -1)
        return read_local_celsius();

    if (index < 0 || index >= m_asicCount)
        return NAN;

    uint8_t ch = index + 1;

    vTaskDelay(pdMS_TO_TICKS(m_wait_after_switch_ms));
    (void)read_remote_celsius(ch);   // dummy read
    vTaskDelay(pdMS_TO_TICKS(m_wait_before_read_ms));

    return temp_correct(ch, read_remote_celsius(ch));
}

// -----------------------------------------------------------------------------
// Calibration
// -----------------------------------------------------------------------------

float TMP468::temp_correct(uint8_t ch, float t)
{
    if (isnan(t) || ch < 1 || ch > 8) return t;

    return ((t - 30.0f) * gCal.scale + 30.0f)
           + gCal.off[ch - 1];
}
