#pragma once

#include <Component/Serde.h>
#include <SerdeGui.h>

namespace VPE {
enum CarGroundConditionFlagBits : uint32_t {
    CAR_GROUND_CONDITION_SLOW_DOWN_BIT = 1u << 0,
    CAR_GROUND_CONDITION_NO_TRAIL_PARTICLES_BIT = 1u << 1,
    CAR_GROUND_CONDITION_SLIPPERY_BIT = 1u << 2,
    CAR_GROUND_CONDITION_SAND_TIRE_MARK_EFFECT_BIT = 1u << 3,
    CAR_GROUND_CONDITION_DIRT_TIRE_MARK_EFFECT_BIT = 1u << 4,
};

using CarGroundConditionFlags = uint32_t;

struct CarGroundCondition {
    CarGroundConditionFlags conditions = 0;

    template <typename Ar>
    void Serde(Ar &&ar) {
        VPE_SERDE_FIELD(conditions);
        if constexpr (IsDebugDrawArchive<Ar>) {
            ar.dirty = DrawInspector() || ar.dirty;
        }
    }

    bool DrawInspector();
};
}  // namespace VPE