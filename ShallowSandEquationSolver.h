#pragma once

#include "GLObjects.h"
#include "GLProgramObject.h"

namespace VPE {
class ShallowSandEquationSolver {
public:
    ShallowSandEquationSolver(float *ground_height,
                              uint32_t height_resolution_x, uint32_t height_resolution_y,
                              float physical_size_x, float physical_size_y);

    GLuint SandVisualizationParitcleBuffer() const;
    uint32_t SandVisualizationParticleCount() const;
    void DrawDebugGui();
    void StepSimulation(double delta_time);
    GLuint GroundHeightMapTexture();

private:
    GLTextureObject m_heightMap{};
    GLTextureObject m_sandSimulationState[2];
    uint32_t m_currentSimulationStateIndex = 0;
    float m_simulationGridSize = 0.0;
    uint32_t m_simulationResolutionX{};
    uint32_t m_simulationResolutionY{};
    uint32_t m_visualParticlePerGrid = 10;
    GLBufferObject m_sandVisualizationParticles{};

    GLProgramObject m_solverProgram{};
    GLBufferObject m_solverParam{};
};
}  // namespace VPE
