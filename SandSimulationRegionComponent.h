#pragma once
#pragma once

#include "../MiniCore/Component/SerdeComponentType.h"
#include <HeightField.h>
#include <GLObjects.h>
#include <GLProgramObject.h>
#include <GLDynamicSizedBuffer.h>
#include "PhysIKAIntegration.h"

struct cudaGraphicsResource;

namespace VPE {
class ShallowSandEquationSolver;
class SandSimulationRegion;
class PhysIKACar;
class SandSimulationRegionComponent {
public:
    struct Data {
        std::shared_ptr<HeightField> height_field{};
        VPE::dvec3 cached_height_field_reference_position{};
        int cached_height_field_level{};
        // alpha for size
        struct Particles {
            // buffer of vec4, encodes {position.xyz, radius}
            GLBufferObject position_buffer{};
            GLBufferObject position_buffer_sorted{};
            cudaGraphicsResource *position_buffer_cuda{};
            size_t buffer_size{};
            size_t particle_count{};

            std::vector<float> radiuses{};
            float *radiuses_d{};

            void EnsureBufferSize(size_t target_size_in_bytes);
            void Resize(size_t particle_count);

            ~Particles();
        };

        Particles sand_particles{};

        std::unique_ptr<ShallowSandEquationSolver> sse_solver;
        std::shared_ptr<SandSimulationRegion> sim_region{};
        SandSolverAlgorithm sand_solver_algorithm = SandSolverAlgorithm::HeightField;
        int sand_height_resolution_x = 0;
        int sand_height_resolution_y = 0;
        int particle_per_grid = 64;
        float grid_physical_size = 0.0f;
        GLBufferObject sand_height_buffer{};
        GLBufferObject particle_noise_buffer{};
        cudaGraphicsResource *sand_height_buffer_cuda{};
        GLBufferObject update_particle_param_buffer{};
    };

    std::shared_ptr<Data> data{};

    int block_size = 1;
    int height_field_quad_tree_level = 24;
    float min_radius = 0.2f;
    float max_radius = 0.4f;
    VPE::vec4 color{1.0f, 0.0f, 0.0f, 1.0f};

    template <typename Ar>
    void Serde(Ar &&ar) {
        VPE_SERDE_FIELD(block_size);
        VPE_SERDE_FIELD(height_field_quad_tree_level);
        VPE_SERDE_FIELD(min_radius);
        VPE_SERDE_FIELD(max_radius);
        VPE_SERDE_FIELD(color);
    }

    void UpdateHeightField(VPE::dvec3 reference_position);
    void DrawGizmos();
    void DrawDebugGui();
    void StartSimulation(const std::vector<PhysIKARigidBodyCreateInfo> &rb_infos);
    void StopSimulation();
    void UpdateSandParticleParticleMethod();
    void UpdateSandParticleHeightFieldMethod(VPE::Entity entity);
    VPE::dmat4 SimulationLocalToWorld();
    VPE::vec3 CenterInSimulationLocal();
    void StepSimulation(VPE::Entity entity, double delta_time);

    //初始化沙子高度
    void InitSandTerrHeight(VPE::dvec3 reference_position);
    //获取地形高度 替换原来的 GetDemData() 接口
    static void GetTerrHeight(const VPE::dvec3& lon_lat, VPE::HeightField* data, int level);

private:
    VPE::dmat4 PatchToSimulationLocal();
    VPE::dmat4 PatchLocalToWorld();
    void InitSandParticles();
    void UpdateSandParticleBuffer(const std::vector<VPE::vec4> &local_pos);
};

void DrawHeightFieldGizmos(HeightField *height_field);

void RegisterSandSimulationRegionComponent();

}  // namespace VPE