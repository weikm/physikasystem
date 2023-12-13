#if 1
#include "PhysIKARegisterComponents.h"
#include "Public/SandSimulationRegionComponent.h"
#include "Component/ComponentRegistry.h"
#include "Public/PolygonRegion.h"
#include "Public/CarGroundCondition.h"

namespace VPE {
void PhysIKARegisterComponents() {
    RegisterSandSimulationRegionComponent();
    ComponentRegistry::MakeAndRegisterComponentType<SerdeComponentType<PolygonRegion, PolygonRegion>>("polygon_region");
    ComponentRegistry::MakeAndRegisterComponentType<SerdeComponentType<CarGroundCondition>>("car_ground_condition");
}
}  // namespace VPE
#endif