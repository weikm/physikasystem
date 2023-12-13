#pragma once
#include "IPlugin.h"

class PhysiKASystemPlugin : VPE::IPlugin {
public:
    PhysiKASystemPlugin(){};
    bool Init(void *lpvoid) override;
    bool Clean() override;
};

extern "C" BOOST_SYMBOL_EXPORT PhysiKASystemPlugin VPEPlugin;
