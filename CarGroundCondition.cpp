#include "CarGroundCondition.h"

namespace VPE {
bool CarGroundCondition::DrawInspector() {
    bool dirty = false;
    auto checkbox = [&](const char *label, CarGroundConditionFlagBits flag) {
        bool checked = conditions & flag;
        if (ImGui::Checkbox(label, &checked)) {
            dirty = true;
            if (checked) {
                conditions |= flag;
            } else {
                conditions &= ~flag;
            }
        }
    };

    checkbox("Slow Down", CAR_GROUND_CONDITION_SLOW_DOWN_BIT);
    checkbox("No Trial Particles", CAR_GROUND_CONDITION_NO_TRAIL_PARTICLES_BIT);
    checkbox("Sliperry", CAR_GROUND_CONDITION_SLIPPERY_BIT);
    checkbox("Sand Tire Mark Effect", CAR_GROUND_CONDITION_SAND_TIRE_MARK_EFFECT_BIT);
    checkbox("Dirt Tire Mark Effect", CAR_GROUND_CONDITION_DIRT_TIRE_MARK_EFFECT_BIT);

    return dirty;
}
}  // namespace VPE