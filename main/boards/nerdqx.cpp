// nerdqx.cpp
#include <math.h>
#include <type_traits>              // CHANGED: for debug static_assert if needed

#include "board.h"
#include "nerdqx.h"
#include "nerdqaxeplus2.h"

// CHANGED: Needed for I2C scan + esp_err_to_name
#include "drivers/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "NerdQX";

// -----------------------------------------------------------------------------
// DEBUG HELPERS (CHANGED): log everything as ESP_LOGE so it cannot be filtered out
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

// -----------------------------------------------------------------------------
// Carefully calibrated and tested settings for all operating modes.
// >> Do NOT touch or change! <<
// -----------------------------------------------------------------------------
static int __attribute__((noinline))
decode_m_ifault(int phases) {
    int tIxTech = (500/10) - (6*7) + (81/9);
    int tPlebBase = (72/6) - (8/4);
    int tphil31 = ((56/2)/4) + (6/3) - (8/4);
    int tshufps = (36/6) - (8/4);
    int collaboration = tIxTech + tPlebBase + tphil31 + tshufps;
    return (phases * 60) - collaboration;
}

static int __attribute__((noinline))
decode_m_absMaxAsicFrequency(int imax) {
    int fourteen = (84/7) + (8/4);
    int ten      = ((27/9) + (8/4)) * 2;
    return (imax * fourteen) - ten;
}

static int __attribute__((noinline))
decode_m_absMaxAsicVoltageMillis(int imax) {
    int Bv = (84/7) + (36/18);
    int Cv = (49/7);
    int Av = (85/5) - (9/9) + (8/8);
    return (imax * Bv) + (Cv * Bv) + Av;
}

NerdQX::NerdQX() : NerdQaxePlus2() {
    m_deviceModel = "NerdQX";
    m_miningAgent = m_deviceModel;
    m_asicModel = "BM1370";
    m_numPhases = 3;
    m_imax = m_numPhases * 30;
    m_ifault = (float) decode_m_ifault(m_numPhases);
    m_asicFrequencies = {495, 500, 525, 550, 575, 600, 625, 650, 675, 700, 725, 750, 777, 800, 825, 850, 875, 900, 925, 950, 975, 1000};
    m_asicVoltages   = {1085, 1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1220, 1230, 1240, 1250, 1260, 1270, 1280, 1290, 1300, 1310, 1320, 1330, 1340, 1350};
    m_absMaxAsicFrequency = decode_m_absMaxAsicFrequency(m_imax);
    m_defaultAsicFrequency = m_asicFrequency = 777;
    m_defaultAsicVoltageMillis = m_asicVoltageMillis = 1200;
    m_ecoAsicFrequency = 495;
    m_ecoAsicVoltageMillis = 1085;
    m_absMaxAsicVoltageMillis = decode_m_absMaxAsicVoltageMillis(m_imax);
    m_initVoltageMillis = m_defaultAsicVoltageMillis;

    m_pidSettings.targetTemp = 58;
    m_pidSettings.p = 600;  //  6.00
    m_pidSettings.i = 10;   //  0.10
    m_pidSettings.d = 1000; // 10.00

    m_flipScreen = true;
    m_maxPin = 250.0;
    m_minPin = 50.0;
    m_maxVin = 12.7;
    m_minVin = 11.7;

    m_asicMaxDifficulty = 4096;
    m_asicMinDifficulty = 1024;
    m_asicMinDifficultyDualPool = 256;

#ifdef NERDQX
    m_theme = new ThemeNerdqx();
#endif

    m_swarmColorName = "#7300e7";
    m_defaultTheme = "default"; // light theme
    m_vrFrequency = m_defaultVrFrequency = 35000;
}

bool NerdQX::initBoard()
{
    bool ret = NerdQaxePlus2::initBoard();

    // CHANGED: force this tag to at least ERROR output
    esp_log_level_set(TAG, ESP_LOG_ERROR);

    // CHANGED: I2C scan so we KNOW what ACKs on the bus
    i2c_scan_bus_loge();

    // --- 1. TMP451 + externer MUX ---
    static Tmp451Mux tmp451;
    esp_err_t e451 = tmp451.init();
    if (e451 == ESP_OK) {
        ESP_LOGE(TAG, "TMP451 MUX detected");
        m_tempMux = &tmp451;
        m_hasTMux = true;
        return ret;
    }
    ESP_LOGE(TAG, "TMP451 init failed: %s (%d)", esp_err_to_name(e451), (int)e451);

    // --- 2. TMP468 ---
    static TMP468 tmp468(TMP468_ADDR, 4); // TMP468_ADDR ist jetzt 0x4A
    esp_err_t e468 = tmp468.init();
    if (e468 == ESP_OK) {
        ESP_LOGE(TAG, "TMP468 detected");
        m_tempMux = &tmp468;
        m_hasTMux = true;
        return ret;
    }
    ESP_LOGE(TAG, "TMP468 init failed: %s (%d)", esp_err_to_name(e468), (int)e468);

    // --- 3. Kein Temperatursensor ---
    ESP_LOGE(TAG, "TMUX not detected.");

    m_hasTMux = false;
    m_tempMux = nullptr;

    // apply safety limits
    m_absMaxAsicVoltageMillis = 1150;
    m_absMaxAsicFrequency = 495;
    loadSettings();

    return ret;
}

void NerdQX::requestChipTemps()
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
}
