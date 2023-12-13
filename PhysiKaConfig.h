#pragma once

#include <string>
#include <vector>
#include "VPMath.h"
#include "JsonUtils.h"

class PhysiKaConfig {
public:
    PhysiKaConfig() noexcept;
    ~PhysiKaConfig() noexcept;

    void Load(const std::string &config_path_ = "PhysikaConfig.json") noexcept;

    const VPE::dvec3 &GetPhysicalOrigin() noexcept { return m_origin; }

    unsigned GetHeightMapPrecisionLevelInTerrainQuadTree() noexcept { return m_terrain_quadtree_level; }

    unsigned GetHeightMapSize() noexcept { return m_height_map_size; }

    float GetPhysicalEngineTimeDelta() noexcept { return m_time_delta; }

    bool UseGPUForPhysicalEngine() noexcept { return m_use_gpu; }

    unsigned GetTerrainGridSize() noexcept { return m_terrain_grid_size; }

    int GetPhysicalSolverStep() noexcept { return m_solver_step; }

    int GetPhysicalContactSolveIter() noexcept { return m_contact_solve_iter; }

    bool IsServer() noexcept { return m_server; }

    const std::string &GetServerIP() { return server_ip; }

    const std::string &GetMultiCastIP() { return multicast_ip; }

    const uint16_t GetServerListenPort() { return server_listen; }

    const uint16_t GetClientListenPort() { return client_listen; }

    const VPE::ivec2 &GetSensorDataSize() { return m_sensor_data_size; }

    const int GetSensorFrameRate() { return m_sensor_frame_rate; }

private:
    void parsePhysikaLocal(VPE::Json::Value &nodes);
    void parsePhysikaNetwork(VPE::Json::Value &nodes);

private:
    VPE::ivec2 m_sensor_data_size;

    VPE::dvec3 m_origin;
    unsigned m_terrain_quadtree_level;
    unsigned m_height_map_size;
    unsigned m_terrain_grid_size;

    float m_time_delta;
    bool m_use_gpu;

    int m_solver_step;
    int m_contact_solve_iter;

    bool m_server = false;

    std::string server_ip;
    std::string multicast_ip;

    uint16_t server_listen;
    uint16_t client_listen;

    int m_sensor_frame_rate;
};
