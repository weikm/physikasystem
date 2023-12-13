#pragma once
#include <vector>

namespace VPE {
    //组件序列化的属性信息
    struct SandSimulateComponentProp {
        int particle_per_grid = 64;
        float grid_physical_size = 0.0f;
        int block_size = 1;
        int height_field_quad_tree_level = 24;
        float min_radius = 0.2f;
        float max_radius = 0.4f;
        VPE::vec4 color{ 1.0f, 0.0f, 0.0f, 1.0f };
        float thickness = 10;
        std::vector<VPE::dvec3> polyRegion;

        template <typename Ar>
        void Serde(Ar&& ar) {
            VPE_SERDE_FIELD(particle_per_grid);
            VPE_SERDE_FIELD(grid_physical_size);
            VPE_SERDE_FIELD(block_size);
            VPE_SERDE_FIELD(height_field_quad_tree_level);
            VPE_SERDE_FIELD(min_radius);
            VPE_SERDE_FIELD(max_radius);
            VPE_SERDE_FIELD(color);
            VPE_SERDE_FIELD(thickness);
            //VPE_SERDE_FIELD(polyRegion);
        }
    };
}