#include "TMP468.h"
#include "i2c_master.h"
#include <esp_check.h>

struct TempCal {
    float scale;
    float off[8]; // Offsets für Remote-Kanäle 1-8, Intern=0
};

static TempCal gCal = {
    1.09f,
    { -29.5f, -29.5f, -29.5f, -29.5f, -29.5f, -29.5f, -29.5f, -29.5f }
};

TMP468::TMP468(uint8_t addr, i2c_port_t port)
    : m_addr(addr), m_port(port) {
    // CHANGE: Default-Sicherheit
    m_asicCount = 8;
}

// -----------------------------------------------------------------------------
// I2C Implementierungen
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
// Initialisierung
// -----------------------------------------------------------------------------

esp_err_t TMP468::init() {
    uint8_t msb, lsb;

    // Hersteller-ID prüfen
    ESP_RETURN_ON_ERROR(read_reg_16(REG_MAN_ID, &msb, &lsb),
                        TAG, "MAN ID read failed");

    uint16_t mid = (msb << 8) | lsb;
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_INVALID_VERSION;
    }

    // Continuous Conversion Mode
    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x00),
                        TAG, "Config write failed");

    // CHANGE: Hardware-Offsets & N-Factors sauber zurücksetzen
    for (int i = 0; i < 8; i++) {
        write_reg(REG_NFACTOR_BASE + (i * 8), 0x00);
        write_reg(REG_OFFSET_BASE  + (i * 8), 0x00);
    }

    ESP_LOGI(TAG, "TMP468 initialized @0x%02X on port %d", m_addr, m_port);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Rohdaten
// -----------------------------------------------------------------------------

bool TMP468::readRawData(uint8_t channel, uint8_t &msb, uint8_t &lsb) {
    // CHANGE: uint8_t kann nicht < 0 sein
    if (channel > 8) return false;
    return (read_reg_16(channel, &msb, &lsb) == ESP_OK);
}

// -----------------------------------------------------------------------------
// Temperatur-Leser
// -----------------------------------------------------------------------------

float TMP468::read_local_celsius() {
    uint8_t msb, lsb;
    if (read_reg_16(0x00, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

float TMP468::read_remote_celsius(int channel) {
    if (channel < 1 || channel > 8) return NAN;
    uint8_t msb, lsb;
    if (read_reg_16(channel, &msb, &lsb) == ESP_OK)
        return make_temp_c(msb, lsb);
    return NAN;
}

// -----------------------------------------------------------------------------
// PUBLIC API – ASIC-basiert
// -----------------------------------------------------------------------------

float TMP468::get_temperature(int asic_index) {
    /*
     * CHANGE:
     * ASIC → TMP468 Mapping
     * ASIC 0 → Channel 1
     */

    if (asic_index < 0 || asic_index >= m_asicCount)
        return NAN;

    int channel = asic_index + 1;

    // ADC Einschwingen
    vTaskDelay(pdMS_TO_TICKS(m_wait_after_switch_ms));

    // Dummy Read (wichtig!)
    (void)read_remote_celsius(channel);

    vTaskDelay(pdMS_TO_TICKS(m_wait_before_read_ms));

    float t_meas = read_remote_celsius(channel);
    return temp_correct(channel, t_meas);
}

// -----------------------------------------------------------------------------
// Kalibrierung
// -----------------------------------------------------------------------------

float TMP468::temp_correct(int ch, float t_meas) {
    if (isnan(t_meas) || ch < 1 || ch > 8)
        return t_meas;

    return ((t_meas - 30.0f) * gCal.scale + 30.0f)
           + gCal.off[ch - 1];
}
