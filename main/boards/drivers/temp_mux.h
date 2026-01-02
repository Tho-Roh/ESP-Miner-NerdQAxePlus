#pragma once
#include "esp_err.h"

class ITempMux {
public:
    virtual ~ITempMux() = default;

    // Detect + init sensor
    virtual esp_err_t init() = 0;

    // channel = ASIC index
    virtual float get_temperature(int channel) = 0;

    // optional helper
    virtual bool readLocalTemp(float* out_C) = 0;
};
