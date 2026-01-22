// tmp468.cpp
#include "tmp468.h"       // NOTE: lower-case include matches your build path
#include <esp_check.h>
#include <math.h>

// Software calibration (wie bei dir)
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

// -----------------------------------------------------------------------------
// I2C helpers (CHANGED: declared in header now)
// -----------------------------------------------------------------------------
esp_err_t TMP468::read_bytes(uint8_t reg, uint8_t* out, size_t len)
{
    return i2c_master_register_read(m_addr, reg, out, len);
}

esp_err_t TMP468::read_word(uint8_t reg, uint16_t* out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = {0, 0};
    esp_err_t err = read_bytes(reg, buf, 2);
    if (err != ESP_OK) return err;
    *out = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
    return ESP_OK;
}

esp_err_t TMP468::write_word(uint8_t reg, uint16_t val)
{
    // writes [reg][MSB][LSB] -> correct for TMP468 word writes
    return i2c_master_register_write_word(m_addr, reg, val);
}

// -----------------------------------------------------------------------------
// Lock / Reset / Busy (CHANGED)
// -----------------------------------------------------------------------------
esp_err_t TMP468::unlock()
{
    return write_word(TMP468_REG_LOCK, TMP468_UNLOCK_KEY);
}

esp_err_t TMP468::soft_reset()
{
    return write_word(TMP468_REG_SOFT_RESET, 0x8000);
}

esp_err_t TMP468::wait_busy_clear(uint32_t timeout_ms)
{
    const TickType_t t0 = xTaskGetTickCount();
    while (true) {
        uint16_t cfg = 0;
        esp_err_t err = read_word(TMP468_REG_CONFIG, &cfg);
        if (err != ESP_OK) return err;

        bool busy = (cfg & (1u << 1)) != 0;
        if (!busy) return ESP_OK;

        if (pdTICKS_TO_MS(xTaskGetTickCount() - t0) > timeout_ms) {
            ESP_LOGE(TAG, "busy timeout, cfg=0x%04X", cfg);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// -----------------------------------------------------------------------------
// Init (CHANGED: word reads/writes, unlock, correct config word)
// -----------------------------------------------------------------------------
esp_err_t TMP468::init()
{
    // 1) Manufacturer ID
    uint16_t mid = 0;
    ESP_RETURN_ON_ERROR(read_word(TMP468_REG_MAN_ID, &mid), TAG, "MAN_ID read failed");
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_NOT_FOUND;
    }

    // 2) Device ID (optional)
    uint16_t did = 0;
    if (read_word(TMP468_REG_DEVICE_ID, &did) == ESP_OK) {
        ESP_LOGI(TAG, "Device ID: 0x%04X", did);
    }

    // 3) Unlock, reset, unlock again
    ESP_RETURN_ON_ERROR(unlock(), TAG, "unlock failed");
    ESP_RETURN_ON_ERROR(soft_reset(), TAG, "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(unlock(), TAG, "unlock-after-reset failed");

    // 4) Write CONFIG (POR 0x0F9C)
    ESP_RETURN_ON_ERROR(write_word(TMP468_REG_CONFIG, TMP468_CONFIG_POR), TAG, "config write failed");

    // 5) Wait until not busy (optional but robust)
    (void)wait_busy_clear(250);

    // 6) Clear offset & n-factor (word writes!)
    for (uint8_t ch = 1; ch <= 8; ch++) {
        (void)write_word(TMP468_OFFSET_REG(ch),  0x0000);
        (void)write_word(TMP468_NFACTOR_REG(ch), 0x0000);
    }

    ESP_LOGI(TAG, "TMP468 initialized @0x%02X", m_addr);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Temperature reads
// -----------------------------------------------------------------------------
float TMP468::read_local_celsius()
{
    uint16_t raw = 0;
    if (read_word(TMP468_REG_TEMP_BASE + 0, &raw) != ESP_OK) return NAN;
    return make_temp_c(raw);
}

float TMP468::read_remote_celsius(uint8_t channel)
{
    if (channel < 1 || channel > 8) return NAN;
    uint16_t raw = 0;
    if (read_word(TMP468_REG_TEMP_BASE + channel, &raw) != ESP_OK) return NAN;
    return make_temp_c(raw);
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
    if (index == -1) return read_local_celsius();
    if (index < 0 || index >= m_asicCount) return NAN;

    uint8_t ch = (uint8_t)(index + 1);
    vTaskDelay(pdMS_TO_TICKS(m_wait_before_read_ms));

    float raw = read_remote_celsius(ch);
    return temp_correct(ch, raw);
}

float TMP468::temp_correct(uint8_t ch, float t_meas)
{
    if (isnan(t_meas) || ch < 1 || ch > 8) return t_meas;
    return ((t_meas - 30.0f) * gCal.scale + 30.0f) + gCal.off[ch - 1];
}

// -----------------------------------------------------------------------------
// Status / Block read
// -----------------------------------------------------------------------------
bool TMP468::readStatusTherm(uint16_t* out_status)
{
    if (!out_status) return false;
    return read_word(TMP468_REG_STATUS_THERM, out_status) == ESP_OK;
}

esp_err_t TMP468::readAllTempsBlock(float outC[9])
{
    if (!outC) return ESP_ERR_INVALID_ARG;

    uint8_t buf[18] = {0};
    esp_err_t err = read_bytes(TMP468_REG_TEMP_BLOCK_BASE, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    for (int i = 0; i < 9; i++) {
        uint16_t raw = (uint16_t(buf[i*2]) << 8) | uint16_t(buf[i*2 + 1]);
        outC[i] = make_temp_c(raw);
    }
    return ESP_OK;
}
