#ifndef TMP468_H
#define TMP468_H

#pragma once

#include <stdint.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "temp_mux.h"

// Projektweiter I2C-Treiber
#include "i2c_master.h"

/*
 * TMP468 – Multi-Channel Temperature Sensor
 *
 * WICHTIG:
 * - Verwendet ausschließlich i2c_master aus dem Projekt
 * - Initialisierung des I2C-Busses erfolgt extern (Board-Level)
 */

#define TMP468_ADDR              0x48
#define TMP468_MANUFACTURER_ID   0x5449

// Register
#define REG_TEMP_BASE            0x00
#define REG_STATUS               0x21  // Device Status Register (kein Temperaturregister!)
#define REG_CONFIG               0x30
#define REG_OFFSET_BASE          0x40
#define REG_NFACTOR_BASE         0x41
#define REG_MAN_ID               0xFE

class TMP468 : public ITempMux {
public:
    // Übergabe des I2C-Ports (Standard: I2C_NUM_0)
    TMP468(uint8_t addr = TMP468_ADDR, i2c_port_t port = I2C_NUM_0);

    // Initialisiert den Sensor (setzt Konfiguration & prüft ID)
    esp_err_t init();

    // channel 0 = lokal, 1–8 = remote
    float get_temperature(int channel);

    // Rohdaten (MSB/LSB) direkt lesen
    bool readRawData(uint8_t channel, uint8_t &msb, uint8_t &lsb);

private:
    uint8_t m_addr;
    i2c_port_t m_port;

    static inline const char* TAG = "TMP468";

    uint32_t m_wait_after_switch_ms = 20;
    uint32_t m_wait_before_read_ms  = 50;

    // I2C-Hilfsfunktionen (i2c_master)
    esp_err_t read_reg(uint8_t reg, uint8_t* out);
    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_reg_16(uint8_t reg, uint8_t* msb, uint8_t* lsb);

    float read_local_celsius();
    float read_remote_celsius(int channel);
    float temp_correct(int ch, float t_meas);

    // TMP468: signed 13-bit Temperatur, LSB = 0.0625°C
    static inline float make_temp_c(uint8_t msb, uint8_t lsb) {
        int16_t raw = (msb << 8) | lsb;
        return static_cast<float>(raw >> 4) * 0.0625f;
    }
};

#endif

