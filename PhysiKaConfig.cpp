#include "PhysiKaConfig.h"
#include "FileSystemUtils.h"
#include "StringUtils.h"
#include "ViWoRoot.h"

PhysiKaConfig::PhysiKaConfig() noexcept
    : m_use_gpu(false), m_time_delta(0.16667), m_height_map_size(3), m_terrain_quadtree_level(16), m_origin(VPE::dvec3(0)) {
}

PhysiKaConfig::~PhysiKaConfig() noexcept {
}

void PhysiKaConfig::Load(const std::string &config_path_) noexcept {
    VPE::ChangeWorkDir();
    std::string file_path = VPE::ConfigDir + config_path_;

    VPE::Json::Document jsonDoc = VPE::Json::ReadJson(file_path);

    if (!jsonDoc.IsNull()) {
        auto itr = jsonDoc.FindMember("PhysiKaConfig");
        if (itr != jsonDoc.MemberEnd()) {
            parsePhysikaLocal(itr->value);
        }
        itr = jsonDoc.FindMember("PhysiKaNetwork");
        if (itr != jsonDoc.MemberEnd()) {
            parsePhysikaNetwork(itr->value);
        }
    }
}

void PhysiKaConfig::parsePhysikaLocal(VPE::Json::Value &jsonDoc) {
    auto itr = jsonDoc.FindMember("PhysicalOrigin");
    if (itr != jsonDoc.MemberEnd()) {
        auto pos = itr->value.GetArray();
        m_origin.x = pos[0].GetDouble();
        m_origin.y = pos[1].GetDouble();
        m_origin.z = pos[2].GetDouble();
    }

    itr = jsonDoc.FindMember("TerrainGirdSize");
    if (itr != jsonDoc.MemberEnd()) {
        m_terrain_grid_size = itr->value.GetUint();
    }
    itr = jsonDoc.FindMember("PhysicalHeightMapLevel");
    if (itr != jsonDoc.MemberEnd()) {
        m_terrain_quadtree_level = itr->value.GetUint();
    }
    itr = jsonDoc.FindMember("PhysicalHeightMapSize");
    if (itr != jsonDoc.MemberEnd()) {
        m_height_map_size = itr->value.GetUint();
    }
    itr = jsonDoc.FindMember("PhysicalTimeStep");
    if (itr != jsonDoc.MemberEnd()) {
        m_time_delta = itr->value.GetFloat();
    }
    itr = jsonDoc.FindMember("PhysicalUseGPU");
    if (itr != jsonDoc.MemberEnd()) {
        m_use_gpu = itr->value.GetBool();
    }
    itr = jsonDoc.FindMember("PhysicalContactSolveIter");
    if (itr != jsonDoc.MemberEnd()) {
        m_contact_solve_iter = itr->value.GetInt();
    }
    itr = jsonDoc.FindMember("PhysicalSolverStep");
    if (itr != jsonDoc.MemberEnd()) {
        m_solver_step = itr->value.GetInt();
    }
}

void PhysiKaConfig::parsePhysikaNetwork(VPE::Json::Value &jsonDoc) {
    auto itr = jsonDoc.FindMember("is_server");
    if (itr != jsonDoc.MemberEnd()) {
        m_server = itr->value.GetBool();
    } else {
        m_server = false;
    }

    itr = jsonDoc.FindMember("server_ip");
    if (itr != jsonDoc.MemberEnd()) {
        server_ip = std::string(itr->value.GetString());
    }

    itr = jsonDoc.FindMember("multicast_ip");
    if (itr != jsonDoc.MemberEnd()) {
        multicast_ip = std::string(itr->value.GetString());
    }

    itr = jsonDoc.FindMember("server_port");
    if (itr != jsonDoc.MemberEnd()) {
        server_listen = itr->value.GetUint();
    }

    itr = jsonDoc.FindMember("client_port");
    if (itr != jsonDoc.MemberEnd()) {
        client_listen = itr->value.GetUint();
    }

    unsigned __width = 400, __height = 300;
    itr = jsonDoc.FindMember("sensor_data_width");
    if (itr != jsonDoc.MemberEnd()) {
        __width = itr->value.GetUint();
    }
    itr = jsonDoc.FindMember("sensor_data_height");
    if (itr != jsonDoc.MemberEnd()) {
        __height = itr->value.GetUint();
    }

    itr = jsonDoc.FindMember("sensor_frame_rate");
    if (itr != jsonDoc.MemberEnd()) {
        m_sensor_frame_rate = itr->value.GetUint();
    } else {
        m_sensor_frame_rate = 20;
    }

    m_sensor_data_size = {__width, __height};
}
