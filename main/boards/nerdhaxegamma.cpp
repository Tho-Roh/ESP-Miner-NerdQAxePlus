// nerdhaxegamma.cpp
#include <math.h>

#include "board.h"
#include "nerdhaxegamma.h"
#include "nerdqaxeplus2.h"

// CHANGED: Needed for I2C scan + esp_err_to_name
#include "drivers/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "NerdHaxeGamma";

// -----------------------------------------------------------------------------
// DEBUG HELPERS: wie bei NerdQX
// -----------------------------------------------------------------------------
static esp_err_t i2c_probe_addr(uint8_t addr7)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static void i2c_scan_bus_loge()
{
    ESP_LOGE(TAG, "I2C scan start...");
    int found = 0;
    for (uint8_t a = 0x03; a < 0x78; a++) {
        if (i2c_probe_addr(a) == ESP_OK) {
            ESP_LOGE(TAG, "I2C device ACK @ 0x%02X", a);
            found++;
        }
    }
    ESP_LOGE(TAG, "I2C scan done. Found=%d", found);
}

// Optional: Rohwerte aus TMP468 loggen (sehr hilfreich fürs Debug)
// Setze auf 0 wenn du es nicht brauchst
#define TMP468_DEBUG_RAW 1

NerdHaxeGamma::NerdHaxeGamma() : NerdQaxePlus2() {
    m_deviceModel = "NerdHaxe-γ";
    m_miningAgent = m_deviceModel;
    m_asicModel = "BM1370";

    // NerdHaxeGamma hat 6 ASICs
    m_asicCount = 6;

    m_numPhases = 4;
    m_imax = 120;
    m_ifault = 105.0;

    // use m_asicVoltage for init
    m_initVoltageMillis = 0;

    m_maxPin = 250.0;
    m_minPin = 75.0;
    m_maxVin = 13.0;
    m_minVin = 11.0;

    m_asicMaxDifficulty = 4096;
    m_asicMinDifficulty = 1024;
    m_asicMinDifficultyDualPool = 256;

#ifdef NERDHAXEGAMMA
    m_theme = new ThemeNerdhaxegamma();
#endif

    m_swarmColorName = "#00e7e2";  // cyan
}

/*
 * initBoard()
 *
 * - ruft Basisklassen-Init auf
 * - I2C Scan (wie NerdQX) zum Debuggen
 * - initialisiert TMP468 (einziger Sensor)
 */
bool NerdHaxeGamma::initBoard()
{
    bool ret = NerdQaxePlus2::initBoard();

    // wie NerdQX: Tag mindestens ERROR, damit Logs nicht gefiltert werden
    esp_log_level_set(TAG, ESP_LOG_ERROR);

    // I2C scan: wir wollen sicher sehen ob 0x4A wirklich ACKt
    i2c_scan_bus_loge();

    // TMP468: nur 6 Kanäle/ASICs benutzen
    static TMP468 tmp468(TMP468_ADDR, 6);

    esp_err_t e468 = tmp468.init();
    if (e468 == ESP_OK) {
        ESP_LOGE(TAG, "TMP468 detected @0x%02X", TMP468_ADDR);
        m_tempMux = &tmp468;
        m_hasTMux = true;
        return ret;
    }

    ESP_LOGE(TAG, "TMP468 init failed: %s (%d)", esp_err_to_name(e468), (int)e468);
    ESP_LOGE(TAG, "Temperature monitoring disabled!");

    m_hasTMux = false;
    m_tempMux = nullptr;

    return ret;
}

/*
 * requestChipTemps()
 *
 * - wie NerdQX: loggt ASIC i temp
 * - ASIC 0 -> TMP468 CH1
 * - ASIC 5 -> TMP468 CH6
 */
void NerdHaxeGamma::requestChipTemps()
{
    // In shutdown the LDOs are not powered and we can't measure chip temps,
    // so reset to 0 to prevent stale values
    if (m_shutdown) {
        for (int i = 0; i < m_asicCount; i++) {
            setChipTemp(i, 0.0f);
        }
        return;
    }

    // don't try when we know we don't have it
    if (!m_hasTMux || !m_tempMux) {
        ESP_LOGE(TAG, "No temperature mux available");
        return;
    }

    for (int i = 0; i < m_asicCount; i++) {
        float temp = m_tempMux->get_temperature(i);
        ESP_LOGE(TAG, "ASIC %d temp = %.2f", i, temp);

        if (!isnan(temp)) {
            setChipTemp(i, temp);
        } else {
            ESP_LOGE(TAG, "ASIC %d temp NAN", i);
        }
    }

#if TMP468_DEBUG_RAW
    // Zusätzliche Rohwert-Checks (optional)
    // Hinweis: Wir können hier nur über i2c_master direkt lesen,
    // weil TMP468::read_word private ist.
    // Remote Temps sitzen bei 0x00..0x08 (word regs)
    for (int ch = 1; ch <= m_asicCount; ch++) {
        uint8_t reg = (uint8_t)(TMP468_REG_TEMP_BASE + ch);

        uint8_t buf[2] = {0};
        esp_err_t err = i2c_master_register_read(TMP468_ADDR, reg, buf, 2);
        if (err == ESP_OK) {
            uint16_t raw = (uint16_t(buf[0]) << 8) | (uint16_t(buf[1]));
            ESP_LOGE(TAG, "TMP468 CH%d raw=0x%04X", ch, raw);
        } else {
            ESP_LOGE(TAG, "TMP468 CH%d raw read failed: %s", ch, esp_err_to_name(err));
        }
    }
#endif
}
