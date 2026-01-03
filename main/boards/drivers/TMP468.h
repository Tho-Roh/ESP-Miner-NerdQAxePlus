#ifndef TMP468_H
#define TMP468_H

#pragma once

#include <stdint.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "temp_mux.h"
#include "i2c_master.h"

/*
 * TMP468 – Multi-Channel Temperature Sensor
 * ESP32-native implementation (continuous conversion)
 */

#define TMP468_ADDR              0x48
#define TMP468_MANUFACTURER_ID   0x5449

// -----------------------------------------------------------------------------
// Register Map (Datasheet / Linux tmp464 driver)
// -----------------------------------------------------------------------------

#define REG_TEMP_BASE            0x00   // Local = 0x00, Remote1..8 = 0x01..0x08
#define REG_STATUS               0x21   // Status / fault flags
#define REG_CONFIG               0x30
#define REG_MAN_ID               0xFE

// Offset / N-Factor (per Remote Channel)
#define TMP468_OFFSET_REG(ch)    (0x40 + ((ch) - 1) * 8)
#define TMP468_NFACTOR_REG(ch)   (0x41 + ((ch) - 1) * 8)

class TMP468 : public ITempMux {
public:
    TMP468(uint8_t addr,
           i2c_port_t port,
           uint8_t asicCount);

    esp_err_t init() override;

    // ASIC index → temperature
// asic_index semantics (matches Tmp451Mux):
//   -1 → local/internal temperature
//   >=0 → ASIC index (mapped to TMP468 remote channels 1..8)
float get_temperature(int asic_index) override;

    // API parity with Tmp451Mux
    bool readLocalTemp(float* out_C) override;

    // ADD: Sensor health/status
    bool readStatus(uint8_t* out_status);

    // Raw register access
    bool readRawData(uint8_t channel, uint8_t &msb, uint8_t &lsb);

private:
    uint8_t m_addr;
    i2c_port_t m_port;
    uint8_t m_asicCount;

    static inline const char* TAG = "TMP468";

    // NOTE: TMP468 läuft im Continuous Mode → nur ADC settling delays
    uint32_t m_wait_after_switch_ms = 20;
    uint32_t m_wait_before_read_ms  = 50;

    esp_err_t read_reg(uint8_t reg, uint8_t* out);
    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_reg_16(uint8_t reg, uint8_t* msb, uint8_t* lsb);

    float read_local_celsius();
    float read_remote_celsius(uint8_t channel);
    float temp_correct(uint8_t ch, float t_meas);

    // TMP468: signed 13-bit, LSB = 0.0625 °C
    static inline float make_temp_c(uint8_t msb, uint8_t lsb) {
        int16_t raw = (msb << 8) | lsb;
        return (raw >> 4) * 0.0625f;
    }
};

#endif

