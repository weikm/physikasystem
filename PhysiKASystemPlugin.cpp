#define NOMINMAX
#include "PhysiKASystemPlugin.h"
#include "ViWoRoot.h"
#include "Plugin/PluginManager.h"
#include "Event/EventManager.h"
#include "Event/KeyBoardEvent.h"
#include "PhysiKASystemBase.h"
#include "PhysIKARegisterComponents.h"

PhysiKASystemPlugin VPEPlugin;

static VPE::PhysIKASystemBase *phy_interface = 0;

bool initPhysikaPluginFromConfig() {
    phy_interface = VPE::CreatePhysIKASystem();
    ViWoROOT::GetModuleManager()->RegistInternalModule(VPE::InternalModuleType::kPhysicsSystem, phy_interface);
    EventManager::RegisterHandler<KeyBoardEvent>(phy_interface);
    VPE::PhysIKARegisterComponents();
    return true;
}

bool cleanPlugin() {
    EventManager::EraseHandler<KeyBoardEvent>(phy_interface);
    SafeDelete(phy_interface);
    return true;
}

bool PhysiKASystemPlugin::Init(void *lpvoid) {
    return initPhysikaPluginFromConfig();
}

bool PhysiKASystemPlugin::Clean() {
    return cleanPlugin();
}