#pragma once

#include <stdint.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "temp_mux.h"
#include "i2c_master.h"

// -----------------------------------------------------------------------------
// TMP468 – Multi-Channel Temperature Sensor (TI)
// Datasheet: SBBA588 / TMP468 Rev. B
// -----------------------------------------------------------------------------

#define TMP468_ADDR                 0x4B
#define TMP468_MANUFACTURER_ID      0x5449

// Register Map
#define TMP468_REG_TEMP_BASE        0x00   // Local=0x00, Remote1..8=0x01..0x08
#define TMP468_REG_STATUS           0x21   // Alarm / fault status (not TMP451 compatible)
#define TMP468_REG_CONFIG           0x30
#define TMP468_REG_MAN_ID           0xFE
#define TMP468_REG_DEVICE_ID        0xFF

#define TMP468_OFFSET_REG(ch)       (0x40 + ((ch) - 1) * 8)
#define TMP468_NFACTOR_REG(ch)      (0x41 + ((ch) - 1) * 8)

class TMP468 : public ITempMux {
public:
    TMP468(uint8_t addr, uint8_t asicCount);

    esp_err_t init() override;

    // ITempMux API
    // index == -1 → Local Sensor
    // index >= 0  → ASIC index → Remote Channel (index + 1)
    float get_temperature(int index) override;
    bool readLocalTemp(float* out_C) override;
    bool readStatus(uint8_t* out_status);

private:
    uint8_t m_addr;
    uint8_t m_asicCount;

    static constexpr const char* TAG = "TMP468";

    // I2C helpers
    esp_err_t read_reg(uint8_t reg, uint8_t* out);
    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_reg_16(uint8_t reg, uint8_t* msb, uint8_t* lsb);

    float read_local_celsius();
    float read_remote_celsius(uint8_t channel);
    float temp_correct(uint8_t ch, float t_meas);

    // TMP468: signed, 13-bit, 0.0625 °C / LSB
    static inline float make_temp_c(uint8_t msb, uint8_t lsb)
    {
        int16_t raw = (msb << 8) | lsb;
        return (raw >> 4) * 0.0625f;
    }

    uint32_t m_wait_after_switch_ms = 20;
    uint32_t m_wait_before_read_ms  = 50;
};
