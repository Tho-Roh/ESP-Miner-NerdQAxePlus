#pragma once

#include "temp_mux.h"
#include "TMP468.h"

/*
 * Dual TMP468 Manager for OctaXEGamma
 *
 * ASIC mapping:
 *   ASIC 0..3 → TMP468 #0 (channels 1..4)
 *   ASIC 4..7 → TMP468 #1 (channels 1..4)
 */

class Tmp468DualMux : public ITempMux {
public:
    Tmp468DualMux(TMP468* s0, TMP468* s1);

    esp_err_t init() override;

    // index == -1 → local temp of sensor 0 (board temp)
    float get_temperature(int index) override;

    bool readLocalTemp(float* out_C) override;

private:
    TMP468* m_sensors[2];
};
