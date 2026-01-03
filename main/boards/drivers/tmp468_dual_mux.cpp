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

    // Einheitliche ITempMux-Semantik
    // index == -1 liefert die interne (Local) Temperatur von TMP468 #0.
    // Damit ist das Verhalten identisch zu NerdHaxeGamma & NerdQX.
    if (index == -1) {
        return m_a->get_temperature(-1);   // TMP468 #0 local sensor
    }

    if (!m_hasA || !m_hasB)
        return NAN;

    if (index < 0 || index >= 8)
        return NAN;

    if (index < 4) {
        return m_a->get_temperature(index);
    }

    return m_b->get_temperature(index - 4);
}

bool Tmp468DualMux::readLocalTemp(float* out_C) {
    if (!out_C || !m_hasA)
        return false;

    // Local temperature = Board / Ambient
    // Wird immer von TMP468 #0 geliefert
    float t = m_a->get_temperature(-1);
    if (isnan(t))
        return false;

    *out_C = t;
    return true;
}
