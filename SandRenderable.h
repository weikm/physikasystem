#pragma once

#include "Renderable.h"
#include <vector>
#include "VPMath.h"
#include "GLProgramObject.h"
#include "GLObjects.h"
#include "Model.h"
#include "Texture/Texture.h"
#include "Geometry/Transform.h"

namespace VPE {
struct SandSimulationRegionComponent;
struct PolygonRegion;

class SandRenderable : public RenderSystem::IRenderable {
public:
    SandRenderable();
    void Render(void *pOptions, double _time, float _deltatime) override;
    void DrawDebugGui() override;
    ~SandRenderable() override;

private:
    bool m_inited = false;
    void EnsureInit();
    void Sort(VPE::Entity entity, SandSimulationRegionComponent &region);
    void GenerateCommands();
    void Render(SandSimulationRegionComponent &region);
    void InitImposterMesh();

    GLBufferObject m_drawParamBuffer{};
    GLProgramObject m_sandDrawImposterProgram{};
    GLProgramObject m_sandDrawPointProgram{};

    static constexpr int QUAD_DRAW_COMMAND = 0;
    static constexpr int POINT_DRAW_COMMAND = 1;
    GLBufferObject m_drawCommandBuffers[2]{};

    GLBufferObject m_sortParamBuffer{};
    GLBufferObject m_sortCounterBuffers[2]{};
    GLProgramObject m_sandSortProgram{};
    GLProgramObject m_sandSortWithMaskProgram{};

    GLBufferObject m_generateDrawCommandParamBuffer{};
    GLProgramObject m_generateDrawCommandProgram{};
    uint32_t m_frameIndex = 0;

    VPE::Texture *m_baseColorTex{};
    VPE::Texture *m_normalTex{};

    GLVertexArrayObject m_imposterVAO{};
    GLBufferObject m_imposterVBO{};
    GLBufferObject m_imposterEBO{};
    uint32_t m_indicesNum{};
    bool m_sortParticles = true;
};

VPE::mat4 CalculateSimulationLocalToMask(VPE::Entity mask_entity,
                                         SandSimulationRegionComponent &sand);

VPE::Entity FindParentWithPolygonRegion(VPE::Entity self);
}  // namespace VPE