#pragma once

#include "asic.h"
#include "bm1370.h"
#include "board.h"
#include "nerdqaxeplus2.h"
#include "nerdoctaxegamma.h"

#include "./drivers/temp_mux.h"
#include "./drivers/tmp451_mux.h"
#include "./drivers/tmp468.h"

class NerdQX : public NerdQaxePlus2 {
protected:
    ITempMux* m_tempMux = nullptr;
    bool m_hasTMux = false;

public:
    NerdQX();
    virtual bool initBoard();
    virtual void requestChipTemps();
};
