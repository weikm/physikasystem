#pragma once
#include "PhysiKASystemBase.h"
#include "PhysiKaConfig.h"

#include <GLObjects.h>

#include "Event/KeyBoardEvent.h"

#include "CommandList.h"
#include "ISensor.h"

#include "PhysiKaNetData.h"
#include "GLProgramObject.h"
#include <Dynamics/RigidBody/Vehicle/MultiWheelCar.h>

// Forward declare things we use.
// The include of CUDA header fucks the file encoding.
namespace PhysIKA {
class HeightFieldRigidDetectionModule;
class PBDSolverNode;
class ParticleSandRigidInteraction;
class MultiHeightFieldRigidDetectionModule;
class PBDCar;
class PBDSolver;
}  // namespace PhysIKA

namespace VSVLink {
class UDP2SocketLink;
class FrameData;
}  // namespace VSVLink

namespace VPE {
class SandRenderable;
class CAxisAlignedBox;
class RigidBody;
class Collider;
class Ray;
class HeightField;
}  // namespace VPE

namespace VPE {

class PhysIKASystem : public PhysIKASystemBase {
public:
    PhysIKASystem();
    virtual ~PhysIKASystem();

public:
    // override func()
    int OnEvent(double _curtime, double _deltatime, void *_userdata) override;

    bool OnKeyDown(uint16_t key);

    bool Handle(KeyBoardEvent &event) override;

    void Render(void *pOptions, double _time, float _deltatime) override;

    void DrawDebugGui() override;

protected:
    void UpdateHeightField();

    void UpdateCarGroundConditions();

    void UpdateViwoCar();

    void UpdateCarControls();

    void initConfig();

    void initRender();

    void initMem();

    void cleanMem();

    void sendEnttToClient();

private:
    void StartSimulation();

    void addFourWheelCar(VPE::Entity entt);

    std::shared_ptr<PhysIKA::PBDCar> addPBDFourWheelCar(VPE::Entity entity,
                                                        std::shared_ptr<PhysIKA::PBDSolver> solver);

    void addMultiWheelCar(VPE::Entity entt);

    void makeCollider(const std::shared_ptr<PhysIKA::TopologyModule> &topo, const PhysIKA::RigidBody2_ptr &obj);

    void recive_net_data_client(const VSVLink::FrameData &data);

    void recive_net_data_server(const VSVLink::FrameData &data);

    void UpdateEntity(VPE::Entity, const PhysIKA::Vector3f &pos, const PhysIKA::Quaternionf &qua);

    void UpdateEntity(VPE::Entity, const VPE::dvec3 &pos, const VPE::dquat &qua);

    void UpdateEntity(VPE::Entity, const VPE::vec3 &scale);

    std::shared_ptr<PhysIKA::HeightFieldRigidDetectionModule> applyHeightFieldDetectionModule();

    void camera_track_moving_vehicle(const VPE::dvec3 &);

    void vehicle_sensor();

    void handle_client_control(const PhysiKaKeyboardCmd *cmd);

    void handle_sensor_control(const PhysiKaEntitySensorCmd *cmd);

    void update_camera();

    void handle_sensor_data(const std::byte *);

private:
    // for sensor START
    GLuint m_sensor_texID;
    GLBufferObject m_sensor_vertex_buf;
    GLVertexArrayObject m_sensor_vertex_vao;
    //GLProgramObject m_sensor_shader;
    bool m_sensor_view = false;
    bool m_sensor_dirty = false;
    VPE::EntityID m_sensor_entt_id = 0;
    unsigned char *m_sensor_data = 0;

    // for sensor END

    VSVLink::UDP2SocketLink *m_linker = 0;
    VSVLink::CommandList m_send_to_client, m_send_to_server;
    bool m_track_moving_car = false;

protected:
    std::unordered_map<VPE::Entity, std::shared_ptr<PhysIKA::HeightFieldRigidDetectionModule>> m_enttToPhyNode;
    std::unordered_map<VPE::Entity, std::shared_ptr<HeightField>> m_enttToHeightField;

    std::unordered_map<VPE::Entity, std::shared_ptr<PhysIKA::PBDCar>> m_enttToPBDCar;
    std::unordered_map<VPE::Entity, std::shared_ptr<PhysIKA::MultiWheelCar<4>>> m_enttToMWCar;

    // nodes
    std::shared_ptr<PhysIKA::PBDSolverNode> m_pbdSolverNode;

    std::shared_ptr<PhysIKA::MultiHeightFieldRigidDetectionModule> m_multiDetectModule;

    VPE::dquat m_globalRot;  // Simulation frame rotation in global coordinate.
    VPE::dvec3 m_viwoOriginGlobalPosition;
    VPE::dvec3 m_viwoOriginGeoPosition;

    PhysiKaConfig m_config;
    bool m_simulateSand = true;
    bool m_drawDebugGizmos = false;

    std::unique_ptr<SandRenderable> m_sandRenderable{};
    bool m_simulationIdle = true;
};
}  // namespace VPE