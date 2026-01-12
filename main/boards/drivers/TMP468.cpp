#include "TMP468.h"
#include <esp_check.h>

// -----------------------------------------------------------------------------
// Software Calibration (matches Tmp451Mux behaviour)
// -----------------------------------------------------------------------------

struct TempCal {
    float scale;
    float off[8];
};

static TempCal gCal = {
    1.09f,
    { -29.5f, -29.5f, -29.5f, -29.5f,
      -29.5f, -29.5f, -29.5f, -29.5f }
};

TMP468::TMP468(uint8_t addr, i2c_port_t port, uint8_t asicCount)
    : m_addr(addr), m_port(port), m_asicCount(asicCount) {}

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

esp_err_t TMP468::read_reg(uint8_t reg, uint8_t* out) {
    return i2c_master_read_reg(m_port, m_addr, reg, out, 1);
}

esp_err_t TMP468::write_reg(uint8_t reg, uint8_t val) {
    return i2c_master_write_reg(m_port, m_addr, reg, &val, 1);
}

esp_err_t TMP468::read_reg_16(uint8_t reg, uint8_t* msb, uint8_t* lsb) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_read_reg(m_port, m_addr, reg, buf, 2);
    if (err != ESP_OK) return err;
    *msb = buf[0];
    *lsb = buf[1];
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------

esp_err_t TMP468::init() {
    uint8_t msb, lsb;

    ESP_RETURN_ON_ERROR(read_reg_16(REG_MAN_ID, &msb, &lsb),
                        TAG, "MAN ID read failed");

    uint16_t mid = (msb << 8) | lsb;
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_INVALID_VERSION;
    }

    // Continuous conversion
    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x00),
                        TAG, "Config write failed");

    // Reset HW calibration (same philosophy as Tmp451Mux)
    for (uint8_t ch = 1; ch <= 8; ch++) {
        write_reg(TMP468_OFFSET_REG(ch), 0x00);
        write_reg(TMP468_NFACTOR_REG(ch), 0x00);
    }

    ESP_LOGI(TAG, "TMP468 initialized @0x%02X", m_addr);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Status / Health
// -----------------------------------------------------------------------------

bool TMP468::readStatus(uint8_t* out_status) {
    if (!out_status) return false;
    return read_reg(REG_STATUS, out_status) == ESP_OK;
}

// -----------------------------------------------------------------------------
// Temperature access
// -----------------------------------------------------------------------------

bool TMP468::readRawData(uint8_t channel, uint8_t &msb, uint8_t &lsb) {
    if (channel < 1 || channel > 8) return false;
    return read_reg_16(REG_TEMP_BASE + channel, &msb, &lsb) == ESP_OK;
}

float TMP468::read_local_celsius() {
    uint8_t msb, lsb;
    if (read_reg_16(REG_TEMP_BASE, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

float TMP468::read_remote_celsius(uint8_t channel) {
    if (channel < 1 || channel > 8) return NAN;
    uint8_t msb, lsb;
    if (read_reg_16(REG_TEMP_BASE + channel, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

bool TMP468::readLocalTemp(float* out_C) {
    if (!out_C) return false;
    float v = read_local_celsius();
    if (isnan(v)) return false;
    *out_C = v;
    return true;
}

// -----------------------------------------------------------------------------
// ASIC / Channel API
// -----------------------------------------------------------------------------

float TMP468::get_temperature(int index) {
    /*
     * FIX / CHANGE:
     * Einheitliche ITempMux-Semantik (wie Tmp451Mux)
     *
     * index == -1 → Local / Internal Sensor
     * index >= 0  → ASIC-Index → Remote Channel (index + 1)
     */

    // --- Local temperature ---
    if (index == -1) {
        return read_local_celsius();
    }

    // --- Remote / ASIC temperature ---
    if (index < 0 || index >= m_asicCount)
        return NAN;

    uint8_t channel = index + 1;  // ASIC 0 → TMP468 Channel 1

    // TMP468 läuft im Continuous Mode,
    // trotzdem kurze Delays für ADC-Einschwingen
    vTaskDelay(pdMS_TO_TICKS(m_wait_after_switch_ms));

    // Dummy-Read (analog TMP451 → erste Messung verwerfen)
    (void)read_remote_celsius(channel);

    vTaskDelay(pdMS_TO_TICKS(m_wait_before_read_ms));

    return temp_correct(channel, read_remote_celsius(channel));
}

// -----------------------------------------------------------------------------
// Calibration
// -----------------------------------------------------------------------------

float TMP468::temp_correct(uint8_t ch, float t) {
    if (isnan(t) || ch < 1 || ch > 8) return t;
    return ((t - 30.0f) * gCal.scale + 30.0f)
           + gCal.off[ch - 1];
}


