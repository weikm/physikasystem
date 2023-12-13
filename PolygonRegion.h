#pragma once

#include <vector>
#include <VPMath.h>
#include <Geometry/Transform.h>
#include "../MiniCore/Component/Serde.h"
#include <SerdeGui.h>
#include <memory>
#include "Event/MouseEventHandler.h"
#include "../MiniCore/Component/SerdeComponentType.h"
#include <Event/KeyBoardEvent.h>

#include <GLObjects.h>

namespace VPE {
struct PolygonVertex {
    VPE::vec2 position;

    template <typename Ar>
    void Serde(Ar &ar) {
        VPE_SERDE_FIELD(position);
    }
};

struct PolygonRegion;

class PolygonRegionSDF {
public:
    PolygonRegionSDF();

    void Generate(const PolygonRegion &polygon);
    int Width() const;
    int Height() const;
    float GridPhysicalSize() const;
    GLuint SDFTextureId() const;

private:
    GLTextureObject m_sdfTex{};
    int m_width = 0;
    int m_height = 0;
    float m_gridPhysicalSize = 0;
    GLBufferObject m_paramBuffer{};
    GLBufferObject m_polygonBuffer{};
};

struct PolygonRegion : VPE::ComponentDataTrait<PolygonRegion> {
    PolygonRegion();

    // Vertices in the local space
    std::vector<PolygonVertex> vertices{};
    VPE::vec2 aabb_min{};
    VPE::vec2 aabb_max{};
    float sdf_grid_size = 0.5f;
    std::shared_ptr<PolygonRegionSDF> sdf{};

    template <typename Ar>
    void Serde(Ar &&ar) {
        VPE_SERDE_FIELD(vertices);
        VPE_SERDE_FIELD(sdf_grid_size);

        if constexpr (IsDebugDrawArchive<Ar>) {
            // Display only, not write to config file
            VPE_SERDE_FIELD(aabb_min);
            VPE_SERDE_FIELD(aabb_max);

            ar.dirty = DrawInspector(ar.entity) || ar.dirty;
        }
    }

    // Do something after deserialization
    static PolygonRegion GetComponent(const PolygonRegion &data, const DeserializeState &state);

    void Commit();
    bool Contains(const VPE::vec2 &v) const;
    void DrawGizmos(const VPE::Transform &local_to_world);
    VPE::dvec3 LocalToWorldPosition(const VPE::Transform &local_to_world, const VPE::vec2 &p);
    VPE::vec2 WorldToLocalPosition(const VPE::Transform &world_to_local, const VPE::dvec3 &pos);
    bool DrawInspector(VPE::Entity entity);

    static void Update();
    static void DrawGizmos();
};

class PolygonRegionEditState : public MouseEventHandler {
public:
    PolygonRegionEditState();

    bool OnLButtonClick(int nFlags, int xPos, int yPos) override;
    bool OnMButtonClick(int nFlags, int xPos, int yPos) override;
    bool OnRButtonClick(int nFlags, int xPos, int yPos) override;
    bool OnMouseMove(int nFlags, int xPos, int yPos) override;

    void SetCurrent(VPE::Entity entity);
    VPE::Entity GetCurrentEntity() const;
    PolygonRegion *GetCurrentPolygonRegion() const;
    int GetMouseX() const;
    int GetMouseY() const;
    bool GetMousePickTerrianPosition(VPE::dvec3 &pos_ws) const;

    static void Init();
    static PolygonRegionEditState *Get();

private:
    VPE::Entity _current_entity{};
    int _mouse_x = 0, _mouse_y = 0;
};
}  // namespace VPE