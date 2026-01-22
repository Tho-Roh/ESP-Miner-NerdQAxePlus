// boards/drivers/tmp468.cpp

#include "tmp468.h"
#include <math.h>
#include <esp_check.h>

// -----------------------------------------------------------------------------
// Optional: Software calibration (derzeit neutral)
// -----------------------------------------------------------------------------
struct TempCal {
    float scale;
    float off[8];
};

static TempCal gCal = {
    1.0f,
    { 0,0,0,0,0,0,0,0 }
};

// -----------------------------------------------------------------------------
TMP468::TMP468(uint8_t addr, uint8_t asicCount)
    : m_addr(addr), m_asicCount(asicCount) {}

// -----------------------------------------------------------------------------
// I2C helpers (TMP468 = 16-bit word-oriented)
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
    return i2c_master_register_write_word(m_addr, reg, val);
}

// -----------------------------------------------------------------------------
// Lock / Reset helpers (TI SBAA588)
// -----------------------------------------------------------------------------
esp_err_t TMP468::unlock()
{
    return write_word(TMP468_REG_LOCK, TMP468_UNLOCK_KEY);
}

esp_err_t TMP468::soft_reset()
{
    // bit15 = soft reset
    return write_word(TMP468_REG_SOFT_RESET, 0x8000);
}

esp_err_t TMP468::wait_busy_clear(uint32_t timeout_ms)
{
    TickType_t t0 = xTaskGetTickCount();

    while (true) {
        uint16_t cfg = 0;
        if (read_word(TMP468_REG_CONFIG, &cfg) != ESP_OK)
            return ESP_FAIL;

        // BUSY-Bit ist laut Datasheet optional → wir nutzen Timeout
        if (pdTICKS_TO_MS(xTaskGetTickCount() - t0) > timeout_ms)
            return ESP_OK;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// -----------------------------------------------------------------------------
// Init sequence (TI Datasheet compliant)
// -----------------------------------------------------------------------------
esp_err_t TMP468::init()
{
    ESP_LOGE(TAG, "TMP468 init start @0x%02X (ASICs=%u)", m_addr, m_asicCount);

    // --- Manufacturer ID ---
    uint16_t man = 0;
    ESP_RETURN_ON_ERROR(
        read_word(TMP468_REG_MAN_ID, &man),
        TAG, "read MAN_ID"
    );

    if (man != TMP468_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "MAN_ID mismatch: 0x%04X", man);
        return ESP_ERR_NOT_FOUND;
    }

    // --- Optional Device ID ---
    uint16_t dev = 0;
    if (read_word(TMP468_REG_DEVICE_ID, &dev) == ESP_OK) {
        ESP_LOGE(TAG, "DEV_ID = 0x%04X", dev);
    }

    // --- Unlock → Reset → Unlock ---
    ESP_RETURN_ON_ERROR(unlock(), TAG, "unlock");
    ESP_RETURN_ON_ERROR(soft_reset(), TAG, "soft reset");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(unlock(), TAG, "unlock after reset");

    // -----------------------------------------------------------------
    // Configuration Register (0x30)
    // Bits 15..8 = REN8..REN1 (Remote Channel Enable)
    // Bits  7..0 = conversion / averaging / flags (POR = 0x9C)
    // -----------------------------------------------------------------
    uint16_t enableMask;
    if (m_asicCount >= 8) {
        enableMask = 0xFF;
    } else {
        enableMask = (uint16_t)((1u << m_asicCount) - 1u);
        // 4 ASICs -> 0x0F
        // 6 ASICs -> 0x3F
    }

    uint16_t cfg = (uint16_t)(enableMask << 8) | 0x009C;

    ESP_RETURN_ON_ERROR(
        write_word(TMP468_REG_CONFIG, cfg),
        TAG, "write CONFIG"
    );

    ESP_LOGE(TAG,
        "TMP468 CONFIG=0x%04X (REN mask=0x%02X)",
        cfg, enableMask
    );

    // Optional settle delay
    (void)wait_busy_clear(250);

    // --- Clear offsets & n-factors ---
    for (uint8_t ch = 1; ch <= 8; ch++) {
        write_word(TMP468_OFFSET_REG(ch),  0x0000);
        write_word(TMP468_NFACTOR_REG(ch), 0x0000);
    }

    ESP_LOGE(TAG, "TMP468 init OK");
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Temperature reading
// -----------------------------------------------------------------------------
float TMP468::read_local_celsius()
{
    uint16_t raw = 0;
    if (read_word(TMP468_REG_TEMP_BASE + 0, &raw) != ESP_OK)
        return NAN;

    return make_temp_c(raw);
}

float TMP468::read_remote_celsius(uint8_t channel)
{
    if (channel < 1 || channel > 8)
        return NAN;

    uint16_t raw = 0;
    if (read_word(TMP468_REG_TEMP_BASE + channel, &raw) != ESP_OK)
        return NAN;

    // TI Datasheet: 0x8000 = invalid / open / short
    if (raw == 0x8000 || raw == 0x0000)
        return NAN;

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
    if (index == -1)
        return read_local_celsius();

    if (index < 0 || index >= m_asicCount)
        return NAN;

    uint8_t ch = (uint8_t)(index + 1);
    vTaskDelay(pdMS_TO_TICKS(m_wait_before_read_ms));

    float raw = read_remote_celsius(ch);
    return temp_correct(ch, raw);
}

float TMP468::temp_correct(uint8_t ch, float t_meas)
{
    if (isnan(t_meas) || ch < 1 || ch > 8)
        return t_meas;

    return ((t_meas - 30.0f) * gCal.scale + 30.0f) + gCal.off[ch - 1];
}

// -----------------------------------------------------------------------------
// Status + Block read
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
        uint16_t raw =
            (uint16_t(buf[i * 2]) << 8) |
             uint16_t(buf[i * 2 + 1]);

        if (raw == 0x0000 || raw == 0x8000)
            outC[i] = NAN;
        else
            outC[i] = make_temp_c(raw);
    }
    return ESP_OK;
}
