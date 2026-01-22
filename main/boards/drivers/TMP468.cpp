// TMP468.cpp
#include "TMP468.h"
#include <esp_check.h>
#include <math.h>

// Software calibration (same model as TMP451) – bleibt wie bei dir
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
// I2C helpers (CHANGED: TMP468 expects 16-bit word register access)
// -----------------------------------------------------------------------------
esp_err_t TMP468::read_bytes(uint8_t reg, uint8_t* out, size_t len)
{
    return i2c_master_register_read(m_addr, reg, out, len);
}

esp_err_t TMP468::read_word(uint8_t reg, uint16_t* out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = {0,0};
    esp_err_t err = read_bytes(reg, buf, 2);
    if (err != ESP_OK) return err;
    *out = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
    return ESP_OK;
}

esp_err_t TMP468::write_word(uint8_t reg, uint16_t val)
{
    // i2c_master_register_write_word sends: [reg][MSB][LSB] which matches TMP468 word writes
    return i2c_master_register_write_word(m_addr, reg, val);
}

// -----------------------------------------------------------------------------
// Lock / Reset / Busy handling (CHANGED: required by datasheet)
// -----------------------------------------------------------------------------
esp_err_t TMP468::unlock()
{
    // Datasheet: unlock by writing 0xEB19 to Lock Register 0xC4
    return write_word(TMP468_REG_LOCK, TMP468_UNLOCK_KEY);
}

esp_err_t TMP468::soft_reset()
{
    // Datasheet: set bit15 of Software Reset reg (0x20) to 1 -> 0x8000
    return write_word(TMP468_REG_SOFT_RESET, 0x8000);
}

esp_err_t TMP468::wait_busy_clear(uint32_t timeout_ms)
{
    // BUSY bit is in CONFIG register bit1 (read-only). CONFIG is 16-bit.  :contentReference[oaicite:6]{index=6}
    const TickType_t t0 = xTaskGetTickCount();
    while (true) {
        uint16_t cfg = 0;
        esp_err_t err = read_word(TMP468_REG_CONFIG, &cfg);
        if (err != ESP_OK) return err;

        bool busy = (cfg & (1u << 1)) != 0;
        if (!busy) return ESP_OK;

        if (pdTICKS_TO_MS(xTaskGetTickCount() - t0) > timeout_ms)
            return ESP_ERR_TIMEOUT;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// -----------------------------------------------------------------------------
// Init (CHANGED: unlock + correct POR config word + reset handling)
// -----------------------------------------------------------------------------
esp_err_t TMP468::init()
{
    // 1) Manufacturer ID check (0xFE -> 0x5449)
    uint16_t mid = 0;
    ESP_RETURN_ON_ERROR(read_word(TMP468_REG_MAN_ID, &mid), TAG, "Manufacturer ID read failed");
    if (mid != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Manufacturer ID mismatch: 0x%04X", mid);
        return ESP_ERR_NOT_FOUND;
    }

    // Optional: Device ID
    uint16_t did = 0;
    if (read_word(TMP468_REG_DEVICE_ID, &did) == ESP_OK) {
        ESP_LOGI(TAG, "TMP468 Device ID: 0x%04X", did);
    }

    // 2) Unlock (device powers up locked; otherwise config writes may be ignored/NACK)  :contentReference[oaicite:7]{index=7}
    ESP_RETURN_ON_ERROR(unlock(), TAG, "Unlock failed");

    // 3) Software reset to force known POR state (optional but robust)  :contentReference[oaicite:8]{index=8}
    ESP_RETURN_ON_ERROR(soft_reset(), TAG, "Soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(5));

    // After reset, lock state may return -> unlock again for safety.
    ESP_RETURN_ON_ERROR(unlock(), TAG, "Unlock after reset failed");

    // 4) Write CONFIG: use datasheet POR 0x0F9C (continuous conversion, all channels enabled)
    // CHANGED: CONFIG is 16-bit; must write full word; old byte-write was wrong.
    ESP_RETURN_ON_ERROR(write_word(TMP468_REG_CONFIG, TMP468_CONFIG_POR), TAG, "Config write failed");

    // Optional: wait until not busy (first conversions start immediately)
    (void)wait_busy_clear(250);

    // 5) Clear N-Factor and Offset for all channels (write full word 0x0000)
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

    // Datasheet note: if read before first conversion for that channel completes -> returns 0x0000  :contentReference[oaicite:9]{index=9}
    // Treat 0 as valid 0°C only if you *expect* 0°C; here we accept it as a number.
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
    if (index == -1) {
        return read_local_celsius();
    }
    if (index < 0 || index >= m_asicCount) {
        return NAN;
    }

    uint8_t ch = (uint8_t)(index + 1);

    // optional tiny wait (no mux switching, but gives ADC time if caller polls quickly)
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
// Status (optional)
// -----------------------------------------------------------------------------
bool TMP468::readStatusTherm(uint16_t* out_status)
{
    if (!out_status) return false;
    return read_word(TMP468_REG_STATUS_THERM, out_status) == ESP_OK;
}

// -----------------------------------------------------------------------------
// Block read: read 0x80..0x88 (18 bytes = 9 temps)  :contentReference[oaicite:10]{index=10}
// -----------------------------------------------------------------------------
esp_err_t TMP468::readAllTempsBlock(float outC[9])
{
    if (!outC) return ESP_ERR_INVALID_ARG;

    uint8_t buf[18] = {0};

    // Pointer 0x80 enables block mode (mirror of 0x00..0x08)  :contentReference[oaicite:11]{index=11}
    esp_err_t err = read_bytes(TMP468_REG_TEMP_BLOCK_BASE, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    for (int i = 0; i < 9; i++) {
        uint16_t raw = (uint16_t(buf[i*2]) << 8) | uint16_t(buf[i*2 + 1]);
        outC[i] = make_temp_c(raw);
    }
    return ESP_OK;
}
