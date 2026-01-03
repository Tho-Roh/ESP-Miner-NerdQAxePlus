#include "tmp468_dual_mux.h"
#include <math.h>

Tmp468DualMux::Tmp468DualMux(TMP468* s0, TMP468* s1) {
    m_sensors[0] = s0;
    m_sensors[1] = s1;
}

esp_err_t Tmp468DualMux::init() {
    if (!m_sensors[0] || !m_sensors[1])
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(m_sensors[0]->init(), "TMP468", "init #0 failed");
    ESP_RETURN_ON_ERROR(m_sensors[1]->init(), "TMP468", "init #1 failed");

    return ESP_OK;
}

float Tmp468DualMux::get_temperature(int index) {
    /*
     * index semantics:
     *   -1 → local temperature (sensor 0)
     *   0..7 → ASIC temperatures
     */

    if (index == -1) {
        return m_sensors[0]->get_temperature(-1);
    }

    if (index < 0 || index >= 8)
        return NAN;

    uint8_t sensor  = index / 4;        // 0 or 1
    uint8_t asicIdx = index % 4;         // 0..3

    return m_sensors[sensor]->get_temperature(asicIdx);
}

bool Tmp468DualMux::readLocalTemp(float* out_C) {
    if (!out_C)
        return false;

    float t = m_sensors[0]->get_temperature(-1);
    if (isnan(t))
        return false;

    *out_C = t;
    return true;
}
