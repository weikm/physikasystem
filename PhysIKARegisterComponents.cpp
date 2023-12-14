#include "PhysIKARegisterComponents.h"
#include "SandSimulationRegionComponent.h"
#include "../MiniCore/Component/ComponentRegistry.h"
#include "PolygonRegion.h"
#include "CarGroundCondition.h"

namespace VPE {
void PhysIKARegisterComponents() {
    RegisterSandSimulationRegionComponent();
    ComponentRegistry::MakeAndRegisterComponentType<SerdeComponentType<PolygonRegion, PolygonRegion>>("polygon_region");
    ComponentRegistry::MakeAndRegisterComponentType<SerdeComponentType<CarGroundCondition>>("car_ground_condition");
}
}  // namespace VPE
