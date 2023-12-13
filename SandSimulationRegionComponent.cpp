#include "Public/SandSimulationRegionComponent.h"

#include <random>

#include "Color.h"
#include "Component/ComponentRegistry.h"
#include "ViWoRoot.h"
#include "VectorDataRender.h"
#include "ITerrain.h"
#include "Public/ShallowSandEquationSolver.h"
#include "Public/PhysIKAIntegration.h"
#include "PhysiKASystemBase.h"
#include <Framework/Framework/SceneGraph.h>
#include <ViwoCar.h>
#include "GLShaderUtils.h"

#include "cuda_gl_interop.h"
#include "SandCudaUtils.h"
#include "Public/PolygonRegion.h"
#include "SandRenderable.h"
#include "PolyRegionCalHelper.h"
#include "log.h"
#include "TerrainDef.h"

namespace VPE {
#if 1
struct SandSimulationRegionComponentTrait : public ComponentDataTrait<SandSimulationRegionComponent> {
    static SandSimulationRegionComponent GetComponent(const DataType &data,
                                                      const DeserializeState &state) {
        auto comp = data;
        comp.data = std::make_shared<SandSimulationRegionComponent::Data>();
        comp.data->height_field = std::make_shared<HeightField>(max(data.block_size, 1));
        return comp;
    }
};
#endif

void SandSimulationRegionComponent::InitSandTerrHeight(VPE::dvec3 reference_position)
{
    auto terrain = ViWoROOT::GetTerrainInterface();
    auto height_field = data->height_field;
    auto grid_size = height_field->GridSize();
    double height_center = height_field->CenterHeight();
    VPE::dvec3 center = height_field->CenterPosInViwoGlobalCoord();
    //auto particle_per_grid = /*10*/;
    //double thickness = /*0.2*/;
    std::vector<VPE::vec4> local_pos{};
    local_pos.reserve(grid_size * grid_size * data->particle_per_grid);

    std::default_random_engine rng;
    std::uniform_real_distribution dist(0.0, 1.0);

    auto patch_to_sim = PatchToSimulationLocal();

    for (int x = 0; x < grid_size; x++)
    {
        for (int y = 0; y < grid_size; y++)
        {
            float height = data->height_field->Data()[y * grid_size + x];
            for (int k = 0; k < data->particle_per_grid; k++)
            {
                float fx = (static_cast<float>(x) + dist(rng) - grid_size / 2.0f) * height_field->DeltaX();
                // Notice the minus sign.
                // The it seems y is indexed from north to south, but our local right hand coordinate's y
                // points from south to north.
                float fy = -(static_cast<float>(y) + dist(rng) - grid_size / 2.0f) * height_field->DeltaY();
                auto r = dist(rng) * (max_radius - min_radius) + min_radius;
                float h = height + thickness * dist(rng) + r; //height + 0.2f * dist(rng);
                VPE::dvec4 patch_pos = { fx, fy, h - height_center, 1.0 };
                auto p = patch_to_sim * patch_pos;
                local_pos.emplace_back(p.x, p.y, p.z, r);
            }
        }
    }
    UpdateSandParticleBuffer(local_pos);
}

void SandSimulationRegionComponent::GetTerrHeight(const VPE::dvec3& lon_lat, VPE::HeightField* data, int level)
{
    if (!data)
        LOG_ERROR << "HeightField is Invalid" << std::endl;

    auto terrain = ViWoROOT::GetTerrainInterface();
    int grid_size = data->GridSize();
    //1. 设置高度场信息
    double height_center = terrain->GetPreciseElevation(lon_lat.x, lon_lat.y, level);
    VPE::dvec3 centerPos(lon_lat.x, lon_lat.y, height_center);
    VPE::dvec3 center;
    g_coord.LongLat2GlobalCoord(centerPos, center);
    data->SetCenterPosInViwoGlobalCoord(center);
    data->SetGeoCenterPosition(centerPos);

    //height->SetGeoResolution()
    VPE::dvec2 lon_lat_size(180.0 * data->DeltaX() * grid_size / (VIWO_EARTHRADIUS * VPE::PI),
        180.0 * data->DeltaY() * grid_size / (VIWO_EARTHRADIUS * VPE::PI));
    data->SetGeoSize(lon_lat_size);
#if 0
    VPE::dvec2 left_top(centerPos.x - lon_lat_size.x / 2, centerPos.y + lon_lat_size.y / 2);
    VPE::dvec2 right_down(centerPos.x + lon_lat_size.x / 2, centerPos.y - lon_lat_size.y / 2);
    VPE::dvec2 delta = right_down - left_top;

    for (int x = 0; x < grid_size; x++)
    {
        for (int y = 0; y < grid_size; y++)
        {
            auto lonlat = left_top + VPE::dvec2(x, y) / VPE::dvec2(grid_size, grid_size) * delta;
            float height = terrain->GetPreciseElevation(lonlat.x, lonlat.y, level);
            data->WriteData(y * grid_size + x, height);
        }
    }
#endif // 0

#if 1
    //高度场在世界下的局部坐标系方向
    VPE::dvec3 up = VPE::normalize(center);
    VPE::dvec3 east = VPE::normalize(VPE::cross(VPE::dvec3(0, 0, 1), up));
    VPE::dvec3 north = VPE::normalize(VPE::cross(up, east));

    VPE::dvec3 center_temp = center - (data->DeltaX() * grid_size / 2) * east +
        (data->DeltaY() * grid_size / 2) * north;
    for (int x = 0; x < grid_size; x++)
    {
        for (int y = 0; y < grid_size; y++)
        {
            double lon, lat, h;
            g_coord.GlobalCoord2LongLat(center_temp, lon, lat, h);
            float height = terrain->GetPreciseElevation(lon, lat, level);
            data->WriteData(y * grid_size + x, height);
            center_temp -= data->DeltaY() * north;
        }
        center_temp = center_temp + data->DeltaY() * grid_size * north + data->DeltaX() * east;
    }
#endif // 0
}

void SandSimulationRegionComponent::UpdateHeightField(VPE::dvec3 reference_position) {
    auto terrain = ViWoROOT::GetTerrainInterface();
    auto &cached_level = data->cached_height_field_level;
    auto &cached_position = data->cached_height_field_reference_position;
    if (!terrain || (cached_level == height_field_quad_tree_level && cached_position == reference_position 
        && data->height_field->Row() == block_size && data->height_field->Col() == block_size)) {
        return;
    }
    //terrain->GetDemData(reference_position, data->height_field.get(), height_field_quad_tree_level);
    cached_position = reference_position;
    cached_level = height_field_quad_tree_level;
    //InitSandParticles();
    GetTerrHeight(reference_position, data->height_field.get(), height_field_quad_tree_level);
    InitSandTerrHeight(reference_position);
}

void SandSimulationRegionComponent::DrawGizmos() {
    DrawHeightFieldGizmos(data->height_field.get(),0.00);
}

void SandSimulationRegionComponent::DrawDebugGui() {
    if (data->sse_solver == nullptr) {
        ImGui::Text("Simulation Not Start");
    } else {
        data->sse_solver->DrawDebugGui();
    }
}

VPE::dmat4 SandSimulationRegionComponent::SimulationLocalToWorld() {
    auto sys = dynamic_cast<PhysIKASystemBase *>(ViWoROOT::GetPhysicsSystem());
    return sys->SimulationRegionLocalToWorld();
}

VPE::vec3 SandSimulationRegionComponent::CenterInSimulationLocal() {
    auto center = data->height_field->CenterPosInViwoGlobalCoord();
    auto center_local = VPE::vec4(VPE::inverse(SimulationLocalToWorld()) * VPE::dvec4(center, 1.0));
    return VPE::vec3(center_local);
}

namespace {
struct UpdateParticleFromHeightFieldParam {
    int resolution_x;
    int resolution_y;
    int particle_per_grid;
    float radius_min;
    VPE::vec3 center_offset;
    float radius_max;
    VPE::vec2 grid_size;
    float _padding[2];
    VPE::mat4 sim_local_to_mask;
};

class UpdateParticleFromHeightProgram {
public:
    UpdateParticleFromHeightProgram() {
        m_updateProgram = OpenGLUtils::CompileAndLinkShaderProgram(
            OpenGLUtils::ShaderConfig()
                .Compute("Sand/UpdateParticleFromHeightField.glsl"));
        m_updateWithMaskProgram = OpenGLUtils::CompileAndLinkShaderProgram(
            OpenGLUtils::ShaderConfig()
                .Compute("Sand/UpdateParticleFromHeightField.glsl")
                .Macro("USE_SDF_MASK", ""));
    }

    static UpdateParticleFromHeightProgram *Get() {
        static auto s_prog = std::make_unique<UpdateParticleFromHeightProgram>();
        return s_prog.get();
    }

    GLuint Id(bool with_mask) {
        if (with_mask) {
            return m_updateWithMaskProgram.get();
        }
        return m_updateProgram.get();
    }

private:
    GLProgramObject m_updateProgram{};
    GLProgramObject m_updateWithMaskProgram{};
};
}  // namespace

void SandSimulationRegionComponent::StartSimulation(const std::vector<PhysIKARigidBodyCreateInfo> &rb_infos) {
    auto height_field = data->height_field;
    auto grid_size = height_field->GridSize();
    auto center_height = height_field->CenterHeight();
    {
        std::vector<float> heights{};
        heights.reserve(grid_size * grid_size);
        for (int x = 0; x < grid_size; x++) {
            for (int y = 0; y < grid_size; y++) {
                float height = data->height_field->Data()[x * grid_size + y] - center_height;
                heights.push_back(height);
            }
        }
        data->sse_solver = std::make_unique<ShallowSandEquationSolver>(heights.data(), grid_size, grid_size,
                                                                       grid_size * height_field->DeltaX(),
                                                                       grid_size * height_field->DeltaY());
    }
    {
        // It seems the PhysIKA sand simulation only supports square grid
        // resample the original height field to make grid square.
        double delta = std::min(height_field->DeltaX(), height_field->DeltaY()) *2.0f;
        int resampled_x = std::ceil(grid_size * height_field->DeltaX() / delta);
        int resampled_y = std::ceil(grid_size * height_field->DeltaY() / delta);

        std::vector<double> resampled_height{};
        resampled_height.resize(resampled_x * resampled_y);

        for (int y = 0; y < resampled_y; y++) {
            for (int x = 0; x < resampled_x; x++) {
                // align center
                auto physical_x = delta * (x - resampled_x * 0.5f);
                auto physical_y = delta * (y - resampled_y * 0.5f);
                auto src_xf = physical_x / height_field->DeltaX() + grid_size * 0.5f;
                auto src_yf = physical_y / height_field->DeltaY() + grid_size * 0.5f;

                auto get_value = [&](int sx, int sy) {
                    sx = std::clamp(sx, 0, grid_size - 1);
                    sy = std::clamp(sy, 0, grid_size - 1);
                    return data->height_field->Data()[sy * grid_size + sx] - center_height;
                };

                auto lerp = [](double a, double b, double f) {
                    return a + (b - a) * f;
                };

                int src_xi = std::floor(src_xf);
                int src_yi = std::floor(src_yf);
                float fx = src_xf - src_xi;
                float fy = src_yf - src_yi;
                auto v00 = get_value(src_xi, src_yi);
                auto v01 = get_value(src_xi, src_yi + 1);
                auto v11 = get_value(src_xi + 1, src_yi + 1);
                auto v10 = get_value(src_xi + 1, src_yi);

                auto rv = lerp(lerp(v00, v01, fy), lerp(v10, v11, fy), fx);

                auto dst_index = y * resampled_x + x;
                resampled_height[dst_index] = rv;
            }
        }
        SandSimulationRegionCreateInfo region_info{};
        region_info.enable_rigid_simulation = false;
        region_info.height_data = resampled_height.data();
        region_info.height_resolution_x = resampled_x;
        region_info.height_resolution_y = resampled_y;
        region_info.grid_physical_size = delta;
        region_info.time_delta = 0.016f;
        region_info.sand_layer_thickness = thickness;
        region_info.rigidbodies = rb_infos;
        region_info.sand_solver_algorithm = SandSolverAlgorithm::HeightField;
        region_info.sand_mu = sand_mu;
        region_info.sand_drag = sand_drag;
        region_info.sand_Rho = sand_Rho;

        auto world = ViWoROOT::World();

        data->sim_region = SandSimulationRegion::Create(region_info);
        data->sand_solver_algorithm = region_info.sand_solver_algorithm;
        data->sand_height_resolution_x = region_info.height_resolution_x;
        data->sand_height_resolution_y = region_info.height_resolution_y;
        data->grid_physical_size = region_info.grid_physical_size;

        // Cuda Height Field buffer
        if (data->sand_height_buffer_cuda != nullptr) {
            cudaGraphicsUnregisterResource(data->sand_height_buffer_cuda);
        }
        if (data->sand_height_buffer.isNull()) {
            data->sand_height_buffer.create();
        }
        glNamedBufferData(data->sand_height_buffer.get(), //替换原来的glNamedBufferStorage接口
                             data->sand_height_resolution_x * data->sand_height_resolution_y * sizeof(float),
                             nullptr,
                             GL_DYNAMIC_DRAW);
        cudaGraphicsGLRegisterBuffer(&data->sand_height_buffer_cuda, data->sand_height_buffer.get(), cudaGraphicsRegisterFlagsNone);

        // particle offset buffer
        if (data->particle_noise_buffer.isNull()) {
            data->particle_noise_buffer.create();
        }
        auto particle_count = data->sand_height_resolution_x * data->sand_height_resolution_y * data->particle_per_grid;
        glNamedBufferData(data->particle_noise_buffer.get(),  //替换原来的glNamedBufferStorage接口
                             particle_count * sizeof(float) * 4,
                             nullptr,
                             GL_DYNAMIC_DRAW);

        std::vector<VPE::vec4> offset{};
        offset.resize(particle_count);

        std::default_random_engine rng;
        std::uniform_real_distribution dist(0.0, 1.0);

        for (auto &p: offset) {
            p.x = dist(rng);
            p.y = dist(rng);
            p.z = dist(rng);
            p.w = dist(rng);
        }

        glNamedBufferSubData(data->particle_noise_buffer.get(), 0, particle_count * sizeof(float) * 4, offset.data());

        if (data->update_particle_param_buffer.isNull()) {
            data->update_particle_param_buffer.create();
        }
        glNamedBufferStorage(data->update_particle_param_buffer.get(),
                             sizeof(UpdateParticleFromHeightFieldParam),
                             nullptr,
                             GL_DYNAMIC_STORAGE_BIT);
    }
}

void SandSimulationRegionComponent::StopSimulation() {
    data->sse_solver = nullptr;
    data->sim_region = nullptr;
}

void SandSimulationRegionComponent::UpdateSandParticleParticleMethod() {
    auto sim_region = data->sim_region.get();
    size_t src_particle_count = 0;
    int particle_split_num = 1;
    auto sand_particles_d = sim_region->GetSandParticlesDevicePtr(src_particle_count);

    auto target_particle_count = particle_split_num * src_particle_count;
    auto &particles = data->sand_particles;
    particles.Resize(target_particle_count);

    static std::default_random_engine rng;
    std::uniform_real_distribution dist(0.0, 1.0);
    bool need_update_device_radius = false;
    while (particles.radiuses.size() < target_particle_count) {
        need_update_device_radius = true;
        particles.radiuses.push_back(dist(rng) * (max_radius - min_radius) + min_radius);
    }
    if (need_update_device_radius) {
        cudaFree(particles.radiuses_d);
        auto buf_size = sizeof(float) * target_particle_count;
        cudaMalloc(&particles.radiuses_d, buf_size);
        cudaMemcpy(particles.radiuses_d,
                   particles.radiuses.data(),
                   buf_size,
                   cudaMemcpyHostToDevice);
    }
    auto center_local = CenterInSimulationLocal();

    cudaGraphicsMapResources(1, &particles.position_buffer_cuda);
    size_t buf_size = 0;
    void *pos_d = nullptr;
    cudaGraphicsResourceGetMappedPointer(&pos_d, &buf_size, particles.position_buffer_cuda);
    if (buf_size >= target_particle_count * sizeof(glm::vec4)) {
        UpdateParticleBufferCuda(src_particle_count,
                                 particle_split_num,
                                 center_local.x, center_local.y, center_local.z,
                                 reinterpret_cast<float *>(pos_d),
                                 sand_particles_d,
                                 particles.radiuses_d);
    }
    cudaGraphicsUnmapResources(1, &particles.position_buffer_cuda);
}

void SandSimulationRegionComponent::UpdateSandParticleHeightFieldMethod(VPE::Entity entity) {
    auto sim_region = data->sim_region.get();
    auto resolution_x = data->sand_height_resolution_x;
    auto resolution_y = data->sand_height_resolution_y;
    auto particle_per_grid = data->particle_per_grid;

    cudaGraphicsMapResources(1, &data->sand_height_buffer_cuda);
    size_t buf_size = 0;
    void *height_d = nullptr;
    cudaGraphicsResourceGetMappedPointer(&height_d, &buf_size, data->sand_height_buffer_cuda);
    size_t src_pitch;
    auto src_height = sim_region->GetSandHeightFieldDevicePtr(src_pitch);
    CopyDoubleToFloatCuda(src_height,
                          src_pitch,
                          reinterpret_cast<float *>(height_d),
                          resolution_x,
                          resolution_y);
    cudaGraphicsUnmapResources(1, &data->sand_height_buffer_cuda);

    auto particle_count = resolution_x * resolution_y * particle_per_grid;
    data->sand_particles.Resize(particle_count);

    auto world = ViWoROOT::World();
    auto mask_entity = FindParentWithPolygonRegion(entity);
    auto mask = world->try_get<PolygonRegion>(mask_entity);


    glUseProgram(UpdateParticleFromHeightProgram::Get()->Id(mask != nullptr));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, data->sand_height_buffer.get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, data->sand_particles.position_buffer.get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, data->particle_noise_buffer.get());

    auto center_local = CenterInSimulationLocal();

    UpdateParticleFromHeightFieldParam param{};
    param.resolution_x = data->sand_height_resolution_x;
    param.resolution_y = data->sand_height_resolution_y;
    param.particle_per_grid = data->particle_per_grid;
    param.radius_max = max_radius;
    param.center_offset = center_local;
    param.radius_min = min_radius;
    param.grid_size = VPE::vec2(data->grid_physical_size, data->grid_physical_size);
    param.sim_local_to_mask = CalculateSimulationLocalToMask(mask_entity, *this);

    glBindBuffer(GL_UNIFORM_BUFFER, data->update_particle_param_buffer.get());
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(UpdateParticleFromHeightFieldParam), &param);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, data->update_particle_param_buffer.get());

    std::vector<GLuint> textures{};
    textures.push_back(data->sse_solver->GroundHeightMapTexture());
    if (mask != nullptr) {
        textures.push_back(mask->sdf->SDFTextureId());
    }
    glBindTextures(0, textures.size(), textures.data());

    glDispatchCompute((resolution_x + 31) / 32, (resolution_y + 31) / 32, particle_per_grid);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

}

void SandSimulationRegionComponent::UpdatePolyRegion(VPE::Entity entity)
{
    auto world = ViWoROOT::World();
    auto mask_entity = FindParentWithPolygonRegion(entity);
    auto mask = world->try_get<PolygonRegion>(mask_entity);

    if (!mask) {
        mask = new PolygonRegion();
    }

    mask->vertices.clear();
    Transform tm_old = world->GetTransform(entity);
    std::vector<VPE::dvec2> tmpVerts;
    for (auto& key : poly_region) {
        PolygonVertex vertex{};
        VPE::vec2 ver_loc = mask->WorldToLocalPosition(tm_old.Inverse(), key);
        vertex.position = ver_loc;
        tmpVerts.emplace_back(ver_loc);
    }

    dvec3 pos_new = mask->LocalToWorldPosition(tm_old, PolyRegionCalHelper::getCentroid(tmpVerts));
    world->SetPosition(entity, pos_new);

    //求多边形区域外接矩形
    VPE::dvec2 outerSquare = PolyRegionCalHelper::getBoundingSquare(tmpVerts);
    //转为经纬度包围盒
    VPE::dvec2 delta = outerSquare / (double)data->height_field->GridSize();
    data->height_field->SetDeltaX(delta.x);
    data->height_field->SetDeltaY(delta.y);
    data->grid_physical_size = std::min(data->height_field->DeltaX(), data->height_field->DeltaY());

    Transform tm_new = world->GetTransform(entity);
    for (auto& key : poly_region) {
        PolygonVertex vertex{};
        VPE::vec2 ver_loc = mask->WorldToLocalPosition(tm_new.Inverse(), key);
        vertex.position = ver_loc;
        mask->vertices.emplace_back(vertex);
    }
    mask->sdf_grid_size = 0.1;
    mask->Commit();

    world->emplace_or_replace<PolygonRegion>(entity, *mask);
}

void SandSimulationRegionComponent::StepSimulation(VPE::Entity entity, double delta_time) {
    if (data->sse_solver != nullptr) {
        data->sse_solver->StepSimulation(delta_time);
    }
    if (data->sim_region != nullptr) {
        switch (data->sand_solver_algorithm) {
        case SandSolverAlgorithm::Particle:
            UpdateSandParticleParticleMethod();
            break;
        case SandSolverAlgorithm::HeightField:
            UpdateSandParticleHeightFieldMethod(entity);
            break;
        }
    }
}

VPE::dmat4 SandSimulationRegionComponent::PatchToSimulationLocal() {
    return VPE::inverse(SimulationLocalToWorld()) * PatchLocalToWorld();
}

VPE::dmat4 SandSimulationRegionComponent::PatchLocalToWorld() {  //
    auto center = data->height_field->CenterPosInViwoGlobalCoord();
    auto up = VPE::normalize(center);
    auto east = VPE::normalize(VPE::cross(VPE::dvec3(0, 0, 1), up));
    auto north = VPE::normalize(VPE::cross(up, east));
    return {
        VPE::dvec4(east, 0.0),
        VPE::dvec4(north, 0.0),
        VPE::dvec4(up, 0.0),
        VPE::dvec4(center, 1.0),
    };
}

#if 0
void SandSimulationRegionComponent::InitSandParticles() {
    auto height_field = data->height_field;
    auto grid_size = height_field->GridSize();
    auto center_height = height_field->CenterHeight();
    std::vector<VPE::vec4> local_pos{};
    local_pos.reserve(grid_size * grid_size * 1);
    std::default_random_engine rng;
    std::uniform_real_distribution dist(0.0, 1.0);
    auto patch_to_sim = PatchToSimulationLocal();
    for (int x = 0; x < grid_size; x++) {
        for (int y = 0; y < grid_size; y++) {
            float height = data->height_field->Data()[y * grid_size + x];
            for (int k = 0; k < 1; k++) {
                float fx = (static_cast<float>(x) + dist(rng) - grid_size / 2.0f) * height_field->DeltaX();
                // Notice the minus sign.
                // The it seems y is indexed from north to south, but our local right hand coordinate's y
                // points from south to north.
                float fy = -(static_cast<float>(y) + dist(rng) - grid_size / 2.0f) * height_field->DeltaY();
                float h = height + 0.2f * dist(rng);
                auto r = dist(rng) * (max_radius - min_radius) + min_radius;
                VPE::dvec4 patch_pos = {fx, fy, h - center_height, 1.0};
                auto p = patch_to_sim * patch_pos;
                local_pos.emplace_back(p.x, p.y, p.z, r);
            }
        }
    }
    UpdateSandParticleBuffer(local_pos);
}
#endif
void SandSimulationRegionComponent::UpdateSandParticleBuffer(const std::vector<VPE::vec4> &local_pos) {
    auto target_size = sizeof(VPE::vec4) * local_pos.size();
    auto &particles = data->sand_particles;
    particles.Resize(local_pos.size());
    if (!local_pos.empty()) {
        glNamedBufferSubData(particles.position_buffer.get(), 0, target_size, local_pos.data());
    }
}

void DrawHeightFieldGizmos(HeightField* height, const double& origin_h) {
	auto gizmos = ViWoROOT::GetVectorDataRender();
	auto left_down = height->GetBottomLeftGeoPos();
	auto top_right = height->GetTopRightGeoPos();
	auto alt = height->CenterHeight();
	gizmos->DrawGeoWireBox(
		VPE::dvec3(left_down, alt - 20.0),
		VPE::dvec3(top_right, alt + 20.0),
		1,
		Color(1.0, 0.0, 0.0, 1.0));
#if 0
	gizmos->DrawGeoGrid(left_down, top_right, alt + 5.0,
		height->GridSize(), height->GridSize(),
		1, Color(0.0, 1.0, 1.0, 1.0));
#endif // 0

#if 0
	//绘制高程点
	int grid_size = height->GridSize();
	VPE::dvec2 left_top(height->GetGeoCenterPosition().x - height->GetGeoSize().x / 2,
		height->GetGeoCenterPosition().y + height->GetGeoSize().y / 2);
	VPE::dvec2 right_down(height->GetGeoCenterPosition().x + height->GetGeoSize().x / 2,
		height->GetGeoCenterPosition().y - height->GetGeoSize().y / 2);

	PointsVector pts{};
	pts.reserve(grid_size * grid_size);
	auto get_point = [&](int x, int y) {
		auto delta = right_down - left_top;
		auto lonlat = left_top + VPE::dvec2(x, y) / VPE::dvec2(grid_size, grid_size) * delta;
		return VPE::dvec3(lonlat, height->Data()[y * grid_size + x] + 0.1);
	};
	for (int i = 0; i < grid_size; i++) {
		for (int j = 0; j < grid_size; j++) {
			pts.push_back(get_point(i, j));
		}
	}
	gizmos->DrawPoints(pts, 2, Color(0.0, 1.0, 1.0, 1.0), CoordMode::kCoordWorldGeo);
#endif // 0
	int grid_size = height->GridSize();
	VPE::dvec3 center = height->CenterPosInViwoGlobalCoord();
	VPE::dvec3 up = VPE::normalize(center);
	VPE::dvec3 east = VPE::normalize(VPE::cross(VPE::dvec3(0, 0, 1), up));
	VPE::dvec3 north = VPE::normalize(VPE::cross(up, east));
	VPE::dvec3 left_top = center - (height->DeltaX() * grid_size / 2) * east +
		(height->DeltaY() * grid_size / 2) * north;

	PointsVector pts{};
	pts.reserve(grid_size * grid_size);
	auto get_point = [&](int x, int y) {
		VPE::dvec3 pos_global = left_top + x * height->DeltaX() * east - y * height->DeltaY() * north;
		double lon, lat, h;
		g_coord.GlobalCoord2LongLat(pos_global, lon, lat, h);
        return VPE::dvec3(lon, lat, height->Data()[y * grid_size + x] + origin_h + 0.05);
	};
	for (int i = 0; i < grid_size; i++) {
		for (int j = 0; j < grid_size; j++) {
			pts.push_back(get_point(i, j));
		}
	}
	gizmos->DrawPoints(pts, 2, Color(0.0, 1.0, 1.0, 1.0), CoordMode::kCoordWorldGeo);
}

#if 1
void RegisterSandSimulationRegionComponent() {
    ComponentRegistry::RegisterComponentType(
        "sand_simulation_region",
        std::make_unique<SerdeComponentType<SandSimulationRegionComponent,
                                            SandSimulationRegionComponentTrait>>());
}
#endif

void SandSimulationRegionComponent::Data::Particles::EnsureBufferSize(size_t target_size_in_bytes) {
    if (target_size_in_bytes > buffer_size) {
        if (position_buffer_cuda != nullptr) {
            cudaGraphicsUnregisterResource(position_buffer_cuda);
        }
        if (position_buffer.isNull()) {
            position_buffer.create();
        }
        if (position_buffer_sorted.isNull()) {
            position_buffer_sorted.create();
        }
        //glNamedBufferStorage(position_buffer.get(), target_size_in_bytes, nullptr, GL_DYNAMIC_STORAGE_BIT);
        glNamedBufferData(position_buffer.get(), target_size_in_bytes, nullptr, GL_DYNAMIC_DRAW);
		//glNamedBufferStorage(position_buffer_sorted.get(), target_size_in_bytes, nullptr, GL_DYNAMIC_STORAGE_BIT);
        glNamedBufferData(position_buffer_sorted.get(), target_size_in_bytes, nullptr, GL_DYNAMIC_DRAW);
        buffer_size = target_size_in_bytes;
        cudaGraphicsGLRegisterBuffer(&position_buffer_cuda, position_buffer.get(), cudaGraphicsRegisterFlagsNone);
    }
}

void SandSimulationRegionComponent::Data::Particles::Resize(size_t particle_count) {
    EnsureBufferSize(particle_count * sizeof(VPE::vec4));
    this->particle_count = particle_count;
}

SandSimulationRegionComponent::Data::Particles::~Particles() {
    if (position_buffer_cuda != nullptr) {
        cudaGraphicsUnregisterResource(position_buffer_cuda);
    }
    cudaFree(radiuses_d);
}
}  // namespace VPE
