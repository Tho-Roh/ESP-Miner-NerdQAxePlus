#include "board.h"
#include "nerdhaxegamma.h"
#include "nerdqaxeplus2.h"

static const char* TAG = "NerdHaxeGamma";

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
 * - initialisiert TMP468
 * - kein Fallback auf TMP451 (Hardware nicht vorhanden)
 */
bool NerdHaxeGamma::initBoard() {
    bool ret = NerdQaxePlus2::initBoard();

    static TMP468 tmp468(TMP468_ADDR, I2C_NUM_0, m_asicCount);

    if (tmp468.init() == ESP_OK) {
        ESP_LOGI(TAG, "TMP468 detected");
        m_tempMux = &tmp468;
        m_hasTMux = true;
        return ret;
    }

    ESP_LOGE(TAG, "TMP468 not detected – temperature monitoring disabled!");
    m_hasTMux = false;
    m_tempMux = nullptr;

    return ret;
}

/*
 * requestChipTemps()
 *
 * - identisch zu NerdQX
 * - liest Temperaturen über ITempMux
 * - Channel-Mapping:
 *      ASIC 0 → TMP468 Channel 1
 *      ASIC 5 → TMP468 Channel 6
 */
void NerdHaxeGamma::requestChipTemps() {

    // Im Shutdown sind LDOs aus → keine Messung möglich
    if (m_shutdown) {
        for (int i = 0; i < m_asicCount; i++) {
            setChipTemp(i, 0.0f);
        }
        return;
    }

    // Kein Sensor vorhanden
    if (!m_hasTMux || !m_tempMux) {
        ESP_LOGE(TAG, "No temperature sensor available");
        return;
    }

    for (int i = 0; i < m_asicCount; i++) {
        float temp = m_tempMux->get_temperature(i);
        // ESP_LOGI(TAG, "temperature of chip %d: %.2f", i, temp);

        if (!isnan(temp)) {
            setChipTemp(i, temp);
        }
    }
}
