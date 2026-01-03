#pragma once

#include "asic.h"
#include "bm1370.h"
#include "board.h"
#include "nerdqaxeplus2.h"
#include "nerdoctaxegamma.h"

// === Temperatur-Sensor-Abstraktion ===
#include "./drivers/temp_mux.h"
#include "./drivers/tmp468.h"

/*
 * NerdHaxeGamma
 *
 * - verwendet ausschließlich TMP468
 * - kein TMP451, kein externer MUX
 * - 6 ASICs → TMP468 Channels 1–6
 */
class NerdHaxeGamma : public NerdQaxePlus2 {
protected:
    ITempMux* m_tempMux = nullptr;  // abstraktes Temperatur-Interface
    bool m_hasTMux = false;         // Sensor vorhanden?

public:
    NerdHaxeGamma();

    // Board-spezifische Initialisierung (TMP468)
    virtual bool initBoard() override;

    // Temperaturabfrage für alle ASICs
    virtual void requestChipTemps() override;
};
