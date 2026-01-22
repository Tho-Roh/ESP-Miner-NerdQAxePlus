// boards/drivers/tmp468.h
#pragma once

#include <stdint.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "temp_mux.h"   // CHANGED: eindeutig derselbe Pfad
#include "i2c_master.h"

// -----------------------------------------------------------------------------
// TMP468 – Multi-Channel Temperature Sensor (TI)
// CHANGED:
//  - I2C address to 0x4A (ADD pin tied to SDA)
//  - TMP468 is word-oriented (16-bit registers) -> use read_word/write_word
//  - CONFIG is 16-bit, POR = 0x0F9C (not 0x009C)
//  - Add LOCK/UNLOCK, SOFT RESET, BLOCK READ base
// -----------------------------------------------------------------------------

#define TMP468_ADDR                 0x4A        // CHANGED
#define TMP468_MANUFACTURER_ID      0x5449

// Register pointers
#define TMP468_REG_TEMP_BASE        0x00        // 00..08: local + remote1..8
#define TMP468_REG_TEMP_BLOCK_BASE  0x80        // CHANGED: 80..88 block-read mirror
#define TMP468_REG_SOFT_RESET       0x20        // CHANGED: 16-bit, bit15=1 resets
#define TMP468_REG_STATUS_THERM     0x21        // optional 16-bit status
#define TMP468_REG_CONFIG           0x30        // CHANGED: 16-bit
#define TMP468_REG_LOCK             0xC4        // CHANGED: 16-bit lock/unlock
#define TMP468_REG_MAN_ID           0xFE        // 16-bit
#define TMP468_REG_DEVICE_ID        0xFF        // 16-bit

#define TMP468_CONFIG_POR           0x0F9C      // CHANGED: correct POR word
#define TMP468_UNLOCK_KEY           0xEB19      // CHANGED
#define TMP468_LOCK_KEY             0x5CA6      // optional

#define TMP468_OFFSET_REG(ch)       (0x40 + ((ch) - 1) * 8)
#define TMP468_NFACTOR_REG(ch)      (0x41 + ((ch) - 1) * 8)

class TMP468 : public ITempMux {
public:
    TMP468(uint8_t addr = TMP468_ADDR, uint8_t asicCount = 8);

    esp_err_t init() override;

    float get_temperature(int index) override;
    bool  readLocalTemp(float* out_C) override;

    // optional helpers
    bool readStatusTherm(uint16_t* out_status);
    esp_err_t readAllTempsBlock(float outC[9]); // CHANGED: block read

private:
    uint8_t m_addr;
    uint8_t m_asicCount;

    static constexpr const char* TAG = "TMP468";

    // CHANGED: TMP468 is word-oriented, so use word I2C helpers
    esp_err_t read_bytes(uint8_t reg, uint8_t* out, size_t len);
    esp_err_t read_word(uint8_t reg, uint16_t* out);
    esp_err_t write_word(uint8_t reg, uint16_t val);

    esp_err_t unlock();
    esp_err_t soft_reset();
    esp_err_t wait_busy_clear(uint32_t timeout_ms);

    float read_local_celsius();
    float read_remote_celsius(uint8_t channel);
    float temp_correct(uint8_t ch, float t_meas);

    static inline float make_temp_c(uint16_t raw16)
    {
        int16_t s = (int16_t)raw16;
        return (s >> 4) * 0.0625f;
    }

    uint32_t m_wait_before_read_ms = 5;
};
