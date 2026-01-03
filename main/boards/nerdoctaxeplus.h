#pragma once

#include "nerdqaxeplus.h"

class NerdOctaxePlus : public NerdQaxePlus {
  public:
    NerdOctaxePlus();
virtual float getVRTemp() override;
virtual bool initBoard() override;
virtual void requestChipTemps() override;
};
