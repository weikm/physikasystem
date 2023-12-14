#include "ShallowSandEquationSolver.h"
#include <imgui/imgui.h>
#include "GLShaderUtils.h"
#include "VPMath.h"

namespace VPE {
namespace {
struct ShallowSandEquationParam {
    int width;
    int height;
};
}  // namespace
ShallowSandEquationSolver::ShallowSandEquationSolver(float *ground_height, uint32_t height_resolution_x, uint32_t height_resolution_y, float physical_size_x, float physical_size_y) {
    m_heightMap.alloc();
    glBindTexture(GL_TEXTURE_2D, m_heightMap.get());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, height_resolution_x, height_resolution_y, 0, GL_RED, GL_FLOAT, ground_height);

    m_simulationGridSize = std::min(
        physical_size_x / float(height_resolution_x),
        physical_size_y / float(height_resolution_y));
    m_simulationResolutionX = physical_size_x / m_simulationGridSize;
    m_simulationResolutionY = physical_size_y / m_simulationGridSize;
    std::vector<VPE::vec4> init_state(m_simulationResolutionX * m_simulationResolutionY, VPE::vec4(0.4f, 0.0f, 0.0f, 1.0f));
    for (auto &sim_state: m_sandSimulationState) {
        sim_state.alloc();
        glBindTexture(GL_TEXTURE_2D, sim_state.get());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_simulationResolutionX, m_simulationResolutionY, 0, GL_RGBA, GL_FLOAT, init_state.data());
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    auto alloc_buffer = [](GLBufferObject &buffer, uint32_t size) {
        buffer.allocIfNull();
        glBindBuffer(GL_UNIFORM_BUFFER, buffer.get());
        glBufferStorage(GL_UNIFORM_BUFFER, size, nullptr, GL_DYNAMIC_STORAGE_BIT);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    };
    alloc_buffer(m_sandVisualizationParticles, 4 * sizeof(float) * SandVisualizationParticleCount());
    alloc_buffer(m_solverParam, sizeof(ShallowSandEquationParam));

    auto conf = OpenGLUtils::ShaderConfig().Compute("Sand/ShallowSandEquation.glsl");
    m_solverProgram = OpenGLUtils::CompileAndLinkShaderProgram(conf);
}

GLuint ShallowSandEquationSolver::SandVisualizationParitcleBuffer() const {
    return m_sandVisualizationParticles.get();
}

uint32_t ShallowSandEquationSolver::SandVisualizationParticleCount() const {
    return m_simulationResolutionX * m_simulationResolutionY * m_visualParticlePerGrid;
}

void ShallowSandEquationSolver::DrawDebugGui() {
    if (ImGui::TreeNode("SSE Solver")) {
        ImGui::Text("Height Map");
        ImGui::Image((ImTextureID)(m_heightMap.get()), {256, 256});
        ImGui::Text("Simulation State 0");
        ImGui::Image((ImTextureID)(m_sandSimulationState[0].get()), {256, 256});
        ImGui::Text("Simulation State 1");
        ImGui::Image((ImTextureID)(m_sandSimulationState[1].get()), {256, 256});
        ImGui::TreePop();
    }
}

void ShallowSandEquationSolver::StepSimulation(double delta_time) {
    glUseProgram(m_solverProgram.get());
    m_currentSimulationStateIndex = (m_currentSimulationStateIndex + 1) % 2;
    GLuint texes[1] = {
        m_heightMap.get(),
    };
    GLuint image_texes[2] = {
        m_sandSimulationState[m_currentSimulationStateIndex].get(),
        m_sandSimulationState[1 - m_currentSimulationStateIndex].get(),
    };
    glBindTextures(0, std::size(texes), texes);
    glBindImageTextures(0, std::size(image_texes), image_texes);

    ShallowSandEquationParam param{};
    param.width = m_simulationResolutionX;
    param.height = m_simulationResolutionY;

    glBindBuffer(GL_UNIFORM_BUFFER, m_solverParam.get());
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(param), &param);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_solverParam.get());

    glDispatchCompute((m_simulationResolutionX + 31) / 32,
                      (m_simulationResolutionY + 31) / 32,
                      1);
}

GLuint ShallowSandEquationSolver::GroundHeightMapTexture() {
    return m_heightMap.get();
}
}  // namespace VPE