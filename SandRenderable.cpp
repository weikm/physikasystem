#include "SandRenderable.h"

#include "Camera.h"
#include "CameraManager.h"
#include "GLShaderUtils.h"
#include "Mesh.h"
#include "RenderSystemInterface.h"
#include "TRenderNode.h"
#include "ViWoRoot.h"
#include "SandSimulationRegionComponent.h"
#include <RenderEngine/IVectorDataRender.h>
#include "ViWoProfile.h"
#include "OpenGLManagedFramebuffer.h"
#include "PolygonRegion.h"

namespace VPE {
namespace {
struct SandDrawParam {
    VPE::mat4 localToView;
    VPE::mat4 viewToHClip;
    VPE::vec4 sandColor;
    VPE::vec3 viewDirWS;
    int isQuad;
    int particleCount;
};

struct DrawCommand {
    GLuint count;
    GLuint instanceCount;
    GLuint firstIndex;
    GLuint baseVertex;
    GLuint baseInstance;
};

struct SortParam {
    VPE::mat4 localToView;
    VPE::mat4 viewToHClip;
    int framebufferWidth;
    int framebufferHeight;
    uint32_t particleCount;
    float _padding;
    VPE::mat4 simulationLocalToMask;
};

struct SortCounters {
    uint32_t quadCount = 0;
    uint32_t pointCount = 0;
};

struct GenerateDrawCommandParam {
    uint32_t indicesStart;
    uint32_t indicesNum;
};

void SetBufferData(GLBufferObject &buf, void *v, size_t size) {
    glBindBuffer(GL_UNIFORM_BUFFER, buf.get());
    glBufferSubData(GL_UNIFORM_BUFFER, 0, size, v);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}
}  // namespace

VPE::mat4 CalculateSimulationLocalToMask(VPE::Entity mask_entity,
                                         SandSimulationRegionComponent &sand) {
    if (mask_entity == VPE::EntityNull) {
        return {};
    }
    auto world = ViWoROOT::World();
    auto mask = world->try_get<PolygonRegion>(mask_entity);
    if (!mask) {
        return {};
    }

    auto world_to_local = world->GetTransform(mask_entity).Inverse();
    auto sim_to_local = VPE::mat4(world_to_local.GetMatrix() * sand.SimulationLocalToWorld());
    auto offset = VPE::translate(VPE::vec3(-mask->aabb_min, 0.0f));

    // Swap y, z
    auto select_component = VPE::mat4(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);

    auto size = VPE::vec2(mask->sdf->Width(), mask->sdf->Height()) * mask->sdf_grid_size;
    auto scale = VPE::scale(1.0f / VPE::vec3(size, 1.0));
    return scale * offset * select_component * sim_to_local;
}

VPE::Entity FindParentWithPolygonRegion(VPE::Entity self) {
    auto world = ViWoROOT::World();
    while (self != VPE::EntityNull) {
        if (world->has<PolygonRegion>(self)) {
            break;
        }
        self = world->GetParent(self);
    }
    return self;
}

SandRenderable::SandRenderable() {
    ViWoROOT::GetRenderSystem()->GetCurrentRender()->AddRenderable(this, "PhysIKASandRenderable");
}

void SandRenderable::Render(void *pOptions, double _time, float _deltatime) {
    EnsureInit();

    auto world = ViWoROOT::World();

    for (auto e : world->view<SandSimulationRegionComponent>()) {
        m_frameIndex = 1 - m_frameIndex;
        auto& region = world->get<SandSimulationRegionComponent>(e);
        if (m_sortParticles) {
            Sort(e, region);
        }
        else {
            SortCounters counters{};
            counters.quadCount = region.data->sand_particles.particle_count;
            counters.pointCount = 0;
            SetBufferData(m_sortCounterBuffers[m_frameIndex], &counters, sizeof(counters));
        }
        GenerateCommands();
        Render(region);
    }

}

void SandRenderable::DrawDebugGui() {
    if (ImGui::CollapsingHeader("Sand Renderable")) {
        ImGui::Checkbox("Sort Particles", &m_sortParticles);

        // Get counter data, not explicitly synchronized and not garenteed to be correct, just for
        // debug visualization purpose
        glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, m_sortCounterBuffers[1 - m_frameIndex].get());
        auto last_frame_counters = (SortCounters *)
            glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(SortCounters), GL_MAP_WRITE_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
        auto draw_quad_count = last_frame_counters->quadCount;
        auto draw_point_count = last_frame_counters->pointCount;
        glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

        ImGui::Text("Draw Quads %d", draw_quad_count);
        ImGui::Text("Draw Points %d", draw_point_count);
    }
}

SandRenderable::~SandRenderable() {}

void SandRenderable::EnsureInit() {
    if (m_inited) {
        return;
    }
    InitImposterMesh();
    auto config = OpenGLUtils::ShaderConfig()
                      .Vertex("Sand/Render.glsl")
                      .Fragment("Sand/Render.glsl");
    m_sandDrawImposterProgram = OpenGLUtils::CompileAndLinkShaderProgram(config);
    config.Macro("SAND_RENDER_POINT", "");
    m_sandDrawPointProgram = OpenGLUtils::CompileAndLinkShaderProgram(config);
    m_sandSortProgram = OpenGLUtils::CompileAndLinkShaderProgram(
        OpenGLUtils::ShaderConfig().Compute("Sand/SortSandParticles.glsl"));
    m_sandSortWithMaskProgram = OpenGLUtils::CompileAndLinkShaderProgram(
        OpenGLUtils::ShaderConfig()
            .Compute("Sand/SortSandParticles.glsl")
            .Macro("USE_SDF_MASK", ""));
    m_generateDrawCommandProgram = OpenGLUtils::CompileAndLinkShaderProgram(
        OpenGLUtils::ShaderConfig().Compute("Sand/FillRenderCommand.glsl"));

    auto alloc_buffer = [](GLBufferObject &buffer, size_t size) {
        buffer.allocIfNull();
        glBindBuffer(GL_UNIFORM_BUFFER, buffer.get());
        glBufferData(GL_UNIFORM_BUFFER, size, nullptr, GL_STATIC_DRAW);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    };
    alloc_buffer(m_drawParamBuffer, sizeof(SandDrawParam));
    for (auto &buf: m_drawCommandBuffers) {
        alloc_buffer(buf, sizeof(DrawCommand));
    }
    for (auto &buf: m_sortCounterBuffers) {
        alloc_buffer(buf, sizeof(SortCounters));
    }
    alloc_buffer(m_sortParamBuffer, sizeof(SortParam));
    alloc_buffer(m_generateDrawCommandParamBuffer, sizeof(GenerateDrawCommandParam));

    m_baseColorTex = dynamic_cast<VPE::Texture *>(
        ViWoROOT::GetTextureManager()->Load(VPE::TextureDir + "PhysIKA/coral_ground_02_diff_1k.jpg"));
    m_normalTex = dynamic_cast<VPE::Texture *>(
        ViWoROOT::GetTextureManager()->Load(VPE::TextureDir + "PhysIKA/coral_ground_02_nor_gl_1k.png"));

    m_inited = true;
}

void SandRenderable::Sort(VPE::Entity entity, SandSimulationRegionComponent &region) {
    VIWO_PROFILE_SCOPE_SAMPLE("Sort");
    auto world = ViWoROOT::World();
    auto mask_entity = FindParentWithPolygonRegion(entity);
    auto mask = world->try_get<PolygonRegion>(mask_entity);

    if (mask != nullptr) {
        glUseProgram(m_sandSortWithMaskProgram.get());
        GLuint mask_sdf = mask->sdf->SDFTextureId();
        glBindTextures(0, 1, &mask_sdf);
    } else {
        glUseProgram(m_sandSortProgram.get());
    }

    auto &particles = region.data->sand_particles.position_buffer;
    auto &particles_sorted = region.data->sand_particles.position_buffer_sorted;

    uint32_t particle_count = region.data->sand_particles.particle_count;

    auto camera = ViWoROOT::GetCameraManager()->GetCurrCamera();
    auto viewMatrix = camera->getViewMatrix();
    auto projMatrix = camera->getProjectMatrixd();

    auto *framebuffers =
        ViWoROOT::GetRenderSystem()->GetCurrentRender()->GetManagedFramebuffer();
    {
        SortParam param{};
        param.localToView = VPE::mat4(viewMatrix * region.SimulationLocalToWorld());
        param.viewToHClip = VPE::mat4(projMatrix);
        param.framebufferWidth = framebuffers->GetBufferWidth();
        param.framebufferHeight = framebuffers->GetBufferHeight();
        param.particleCount = particle_count;
        param.simulationLocalToMask = CalculateSimulationLocalToMask(mask_entity, region);
        SetBufferData(m_sortParamBuffer, &param, sizeof(param));
    }
    auto &cur_counter_buffer = m_sortCounterBuffers[m_frameIndex];
    SortCounters counters{};
    SetBufferData(cur_counter_buffer, &counters, sizeof(counters));

    glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_sortParamBuffer.get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particles.get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, particles_sorted.get());
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, cur_counter_buffer.get());

    glDispatchCompute((particle_count + 1023) / 1024, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void SandRenderable::GenerateCommands() {  // Generate Draw Command
    glUseProgram(m_generateDrawCommandProgram.get());
    {
        GenerateDrawCommandParam param{};
        param.indicesNum = m_indicesNum;
        param.indicesStart = 0;
        SetBufferData(m_generateDrawCommandParamBuffer, &param, sizeof(GenerateDrawCommandParam));
    }
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_generateDrawCommandParamBuffer.get());
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, m_sortCounterBuffers[m_frameIndex].get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_drawCommandBuffers[QUAD_DRAW_COMMAND].get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_drawCommandBuffers[POINT_DRAW_COMMAND].get());
    glDispatchCompute(1, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void SandRenderable::Render(SandSimulationRegionComponent &region) {
    VIWO_PROFILE_SCOPE_SAMPLE("Render");
    auto camera = ViWoROOT::GetCameraManager()->GetCurrCamera();
    auto world = ViWoROOT::World();
    auto viewMatrix = camera->getViewMatrix();
    auto projMatrix = camera->getProjectMatrixd();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    auto render = [&](bool point_render,
        GLuint particle_buffer) {
            glUseProgram(point_render ? m_sandDrawPointProgram.get()
                : m_sandDrawImposterProgram.get());
            glBindVertexArray(m_imposterVAO.get());
            glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_drawParamBuffer.get());
            glBindBufferBase(GL_UNIFORM_BUFFER, 1, m_sortCounterBuffers[m_frameIndex].get());
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particle_buffer);

            SandDrawParam param{};
            param.localToView = VPE::mat4(viewMatrix * region.SimulationLocalToWorld());
            param.viewToHClip = VPE::mat4(projMatrix);
            param.sandColor = region.color;
            param.viewDirWS = normalize(VPE::vec3(camera->getDirection()));
            param.isQuad = point_render ? 0 : 1;
            param.particleCount = region.data->sand_particles.particle_count;
            glBindBuffer(GL_UNIFORM_BUFFER, m_drawParamBuffer.get());
            glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(SandDrawParam), &param);
            glBindBuffer(GL_UNIFORM_BUFFER, 0);

            GLuint texes[1] = {
                m_baseColorTex->GetTextureID(),
            };
            auto cmd = point_render ? POINT_DRAW_COMMAND : QUAD_DRAW_COMMAND;
            glBindBuffer(GL_DRAW_INDIRECT_BUFFER, m_drawCommandBuffers[cmd].get());
            glBindTextures(0, std::size(texes), texes);

            glDrawElementsIndirect(point_render ? GL_POINTS : GL_TRIANGLES, GL_UNSIGNED_INT, 0);
    };

    GLuint buf = m_sortParticles ? region.data->sand_particles.position_buffer_sorted.get()
        : region.data->sand_particles.position_buffer.get();
    render(false, buf);
    render(true, buf);

}

void SandRenderable::InitImposterMesh() {
    m_imposterVAO.alloc();
    glBindVertexArray(m_imposterVAO.get());

    m_imposterVBO.alloc();
    glBindBuffer(GL_ARRAY_BUFFER, m_imposterVBO.get());
    VPE::vec2 texCoords[4] = {
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {1.0f, 1.0f},
        {0.0f, 1.0f},
    };
    glBufferData(GL_ARRAY_BUFFER, std::size(texCoords) * sizeof(VPE::vec2), texCoords, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    m_imposterEBO.alloc();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_imposterEBO.get());
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, std::size(indices) * sizeof(uint32_t), indices, GL_STATIC_DRAW);

    m_indicesNum = std::size(indices);
}
}  // namespace VPE
