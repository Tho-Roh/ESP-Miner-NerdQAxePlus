#include "tmp468_dual_mux.h"
#include <math.h>
#include "esp_log.h"

static const char* TAG = "Tmp468DualMux";

Tmp468DualMux::Tmp468DualMux(TMP468* s0, TMP468* s1) {
    m_sensors[0] = s0;
    m_sensors[1] = s1;
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------

esp_err_t Tmp468DualMux::init() {
    if (!m_sensors[0] || !m_sensors[1])
        return ESP_ERR_INVALID_ARG;

    // Both sensors MUST be present
    ESP_RETURN_ON_ERROR(m_sensors[0]->init(), TAG, "TMP468 #0 init failed");
    ESP_RETURN_ON_ERROR(m_sensors[1]->init(), TAG, "TMP468 #1 init failed");

    ESP_LOGI(TAG, "Dual TMP468 initialized successfully");
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Temperature API
// -----------------------------------------------------------------------------

float Tmp468DualMux::get_temperature(int index) {
    /*
     * index semantics:
     *   -1 → Board temperature (MAX of both internal sensors)
     *   0..7 → ASIC temperatures
     */

    // -----------------------------------------------------------------
    // Board / internal temperature
    // -----------------------------------------------------------------
    if (index == -1) {
        float t0 = m_sensors[0]->get_temperature(-1);
        float t1 = m_sensors[1]->get_temperature(-1);

        // Debug visibility of both sensors
        ESP_LOGD(TAG,
                 "TMP468 local temps: A=%.2f°C B=%.2f°C",
                 t0, t1);

        // Robust fallback logic
        if (isnan(t0)) return t1;
        if (isnan(t1)) return t0;

        // Safety policy: use the hotter sensor
        return fmaxf(t0, t1);
    }

    // -----------------------------------------------------------------
    // ASIC temperatures
    // -----------------------------------------------------------------
    if (index < 0 || index >= 8)
        return NAN;

    uint8_t sensor  = index / 4;   // 0 or 1
    uint8_t asicIdx = index % 4;   // 0..3

    return m_sensors[sensor]->get_temperature(asicIdx);
}

// -----------------------------------------------------------------------------
// Local temperature helper
// -----------------------------------------------------------------------------

bool Tmp468DualMux::readLocalTemp(float* out_C) {
    if (!out_C)
        return false;

    float t0 = m_sensors[0]->get_temperature(-1);
    float t1 = m_sensors[1]->get_temperature(-1);

    if (isnan(t0) && isnan(t1))
        return false;

    if (isnan(t0))      *out_C = t1;
    else if (isnan(t1)) *out_C = t0;
    else                *out_C = fmaxf(t0, t1);

    return true;
}
