#include "board.h"
#include "nerdoctaxegamma.h"
#include "nerdqaxeplus2.h"
#include "./drivers/TPS53667.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "./drivers/TMP468.h"
#include "./drivers/tmp468_dual_mux.h"

static TMP468 tmp468_a(0x48, I2C_NUM_0, 4);
static TMP468 tmp468_b(0x49, I2C_NUM_0, 4);

static Tmp468DualMux tempMux(&tmp468_a, &tmp468_b);

static const char* TAG = "nerdoctaxegamma";

NerdOctaxeGamma::NerdOctaxeGamma() : NerdQaxePlus2() {
    m_deviceModel = "NerdOCTAXE-γ";
    m_miningAgent = m_deviceModel;
    m_asicModel = "BM1370";
    m_asicCount = 8;

    m_asicMaxDifficulty = 4096;
    m_asicMinDifficulty = 1024;
    m_asicMinDifficultyDualPool = 256;

     // use m_asicVoltage for init
    m_initVoltageMillis = 0;

    m_maxVin = 13.0;
    m_minVin = 11.0;

#ifdef NERDOCTAXEGAMMA
    m_theme = new ThemeNerdoctaxegamma();
#endif

    m_swarmColorName = "#11d51e"; // green

    // Hardware voltage regulator detection (available from rev 3.0+)
    // GPIO3 is located next to GPIO10 (TPS_EN) on the board for easy routing
    // The pin has internal pull-down, so older boards without the strapping
    // resistor will default to TPS53647 (backward compatibility)
    gpio_reset_pin(VR_DETECT_PIN);
    gpio_set_direction(VR_DETECT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(VR_DETECT_PIN, GPIO_PULLDOWN_ONLY);

    // Allow pin to settle after configuration
    vTaskDelay(pdMS_TO_TICKS(1));

    bool isTPS53667 = gpio_get_level(VR_DETECT_PIN);
    m_isTPS53667 = isTPS53667;  // Save for later use in getVRTemp()

    if (isTPS53667) {
        // TPS53667 configuration: 6 phases, 240A capability
        m_numPhases = 6;
        m_imax = 240;        // Current hardware: 24.9kΩ → 240A max (40A per phase with 6 phases)
        m_ifault = 235.0;
        m_maxPin = 300.0;    // ~300W output Typically
        m_minPin = 30.0;
        m_tps = new TPS53667();

        // Extended frequency range for TPS53667 (6 phases, higher power capacity)
        m_asicFrequencies = {525, 550, 575, 600, 625, 650, 675, 700, 725, 750, 775, 800};
        m_absMaxAsicFrequency = 850;  // Absolute max for manual input (danger zone)

        // Extended voltage range for TPS53667 (6 phases, higher current capacity)
        m_asicVoltages = {1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210, 1220, 1230, 1240, 1250, 1260};

        // Set higher default values for 6-phase configuration
        m_defaultAsicFrequency = m_asicFrequency = 700;
        m_defaultAsicVoltageMillis = m_asicVoltageMillis = 1210;  // 1.21V

        ESP_LOGI(TAG, "TPS53667 voltage regulator detected (GPIO3=HIGH, 6 phases, 240A max with 24.9kΩ resistor)");
    } else {
        // TPS53647 detected or pin not connected (older revisions)
        // Configure for 4-phase operation (uses inherited frequency limits)
        m_numPhases = 4;
        m_imax = 180;        // 33.2kΩ → 180A max (45A per phase with 4 phases)
        m_ifault = 140.0;
        m_maxPin = 200.0;
        m_minPin = 100.0;
        // m_asicFrequencies and m_absMaxAsicFrequency inherited from parent (500-600 MHz, max 800)

        ESP_LOGI(TAG, "TPS53647 voltage regulator detected (GPIO3=LOW, 4 phases, using inherited)");
    }

}

float NerdOctaxeGamma::getVRTemp() {
    // Get temperature from parent implementation
    float vrTemp = NerdQaxePlus::getVRTemp();

    // Apply +8°C offset only for TPS53667 (6 phases) to correct sensor deviation
    if (m_isTPS53667) {
        vrTemp += 8.0f;
    }

    return vrTemp;
}

/*
 * initBoard()
 *
 * - ruft Basisklassen-Init auf
 * - initialisiert TMP468
 * - kein Fallback auf TMP451 (Hardware nicht vorhanden)
 */
bool NerdOctaxeGamma::initBoard() {
    bool ret = NerdQaxePlus2::initBoard();

    if (tempMux.init() == ESP_OK) {
        ESP_LOGI(TAG, "Dual TMP468 detected (both sensors OK)");
        m_tempMux = &tempMux;
        m_hasTMux = true;
    } else {
        ESP_LOGE(TAG, "Dual TMP468 init failed – BOTH sensors required!");
        m_tempMux = nullptr;
        m_hasTMux = false;
    }

    return ret;
}

/*
 * requestChipTemps()
 *
 * - identisch zu NerdQX / NerdHaxeGamma
 * - liest Temperaturen über ITempMux
 * - Dual-TMP468 Mapping (intern):
 *      ASIC 0–3 → TMP468 A (CH1–4)
 *      ASIC 4–7 → TMP468 B (CH1–4)
 */
void NerdOctaxeGamma::requestChipTemps() {

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
