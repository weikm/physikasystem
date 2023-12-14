#include "VSVCmdBuffer.h"
#include "UDP2SocketLink.h"

#include "PhyCommandDecoder.h"

#include "PhysicsSystem.h"
#include "PhysiKASystem.h"
#include "PhysIKAIntegration.h"

#include "ViWoRoot.h"
#include "World.h"
#include "ITerrain.h"
#include "Coordination.h"
#include "TRenderNode.h"
#include "Model.h"
#include "ModelParser.h"
#include "Config/CoreConfig.h"
#include "StringUtils.h"

#include "Camera.h"
#include "CameraManager.h"
#include "MotionManager.h"
#include "DefaultCameraMover.h"

#include "HeightField.h"
#include "ViWoCar.h"

#include "RenderInterface.h"
#include "OpenGLManagedFramebuffer.h"
#include "GLShaderUtils.h"

#include "FreeImagePlus.h"
#include "MemSimplePool.h"
#include "GLResourceStack.h"

#include "World.h"
#include "Ecs/Entity.h"

#include <imgui/imgui.h>
#include <random>
#include "ViWoProfile.h"

#include "SandRenderable.h"
#include "SandSimulationRegionComponent.h"
#include "Framework/Framework/SceneGraph.h"
#include "Dynamics/RigidBody/Vehicle/PBDCar.h"
#include "Dynamics/RigidBody/Vehicle/MultiWheelCar.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "Dynamics/RigidBody/PBDRigid/PBDSolverNode.h"
#include "Dynamics/RigidBody/PBDRigid/MultiHeightFieldRigidDetectionModule.h"
#include "Dynamics/RigidBody/PBDRigid/HeightFieldRigidDetectionModule.h"
#include "Dynamics/Sand/ParticleSandRigidInteraction.h"
#include "Dynamics/Sand/SandSimulator.h"
#include "RenderEngine/IVectorDataRender.h"

#include "PolygonRegion.h"
#include "CarGroundCondition.h"

static const double dis_X = 30.0;
static const double dis_Y = 10.0;
static double dis_Factor = 1.0;

static const int monitor_frame_rate = 60;

namespace VPE {

PhysIKASystem::PhysIKASystem() {
    initConfig();

    initRender();

    initMem();

    PolygonRegionEditState::Init();

   
}

PhysIKASystem::~PhysIKASystem() {
    cleanMem();

    glDeleteBuffers(1, &m_sensor_texID);
}

int PhysIKASystem::OnEvent(double _curtime, double _deltatime, void *_userdata) {
    VIWO_PROFILE_SCOPE_SAMPLE("PhysIKA System");
    bool simulationStart = m_simulationIdle && EngineWorking();
    bool simulationStop = !m_simulationIdle && !EngineWorking();
    m_simulationIdle = !EngineWorking();
    // Only in editor mode should we update center
    auto world = ViWoROOT::World();

    if (simulationStop) {
        for (auto e: world->view<SandSimulationRegionComponent>()) {
            auto &region = world->get<SandSimulationRegionComponent>(e);
            region.StopSimulation();
        }
    }

    if (!EngineWorking()) {
        for (auto e: world->view<SandSimulationRegionComponent>()) {
            auto &region = world->get<SandSimulationRegionComponent>(e);
            region.UpdateHeightField(world->GetGeoPosition(e));
        }
        return 0;
    }

    if (simulationStart) {
        StartSimulation();
    }

    {
        VIWO_PROFILE_SCOPE_SAMPLE("Take One Frame");
        PhysIKA::SceneGraph::getInstance().takeOneFrame();
    }
    UpdateHeightField();
    UpdateCarGroundConditions();
    UpdateCarControls();
    UpdateViwoCar();
    update_camera();
    sendEnttToClient();

    {
        VIWO_PROFILE_SCOPE_SAMPLE("Simulation Region Sync");
        for (auto e: world->view<SandSimulationRegionComponent>()) {
            auto &region = world->get<SandSimulationRegionComponent>(e);
            region.StepSimulation(e, _deltatime);
        }
    }

    PolygonRegion::Update();
    return 0;
}

bool PhysIKASystem::OnKeyDown(uint16_t key) {
    char val;
    switch (key) {
    case 265:
        val = 38;
        break;
    case 264:
        val = 40;
        break;
    case 263:
        val = 37;
        break;
    case 262:
        val = 39;
        break;
    default:
        val = key;
        break;
    }

    KeyBoardEvent event_(KeyBoardEvent::kKeyDown, val, 0);
    return Handle(event_);
}

bool PhysIKASystem::Handle(KeyBoardEvent &event) {
    if (event.GetType() == KeyBoardEvent::kKeyUp)
        return false;

    // key ;
    if (event.GetKey() == -70)
        StartPause();

    if (m_config.IsServer()) {
        return true;
    }

    if (m_all_cars.size() == 0)
        return false;

    m_send_to_server.Reset();

    PhysiKaKeyboardCmd keyCmd;
    keyCmd.entity_id = ViWoROOT::World()->GetID(m_all_cars[m_control_id]);
    keyCmd.key_val = event.GetKey();

    VSVLink::ConstBuffer buf(&keyCmd, sizeof(keyCmd));

    switch (event.GetKey()) {
    case 38:
    case 40:
    case 37:
    case 39:
        handle_client_control(reinterpret_cast<const PhysiKaKeyboardCmd *>(buf.data));
        m_send_to_server.Append(buf);
        break;
    case 'q':
    case 'Q':
        m_control_id = (m_control_id + 1) % m_all_cars.size();
        break;
    case 'n':
    case 'N':
        if (dis_Factor > 0.3) {
            dis_Factor -= 0.01;
        }
        break;
    case 'm':
    case 'M':
        if (dis_Factor < 1.5) {
            dis_Factor += 0.01;
        }
        break;
    case 't':
    case 'T':
        m_track_moving_car = !m_track_moving_car;
        break;
    case 's':
    case 'S':
        m_sensor_view = !m_sensor_view;
        vehicle_sensor();
        break;
    }
    m_linker->Send(m_send_to_server.GetBuffer());
    return true;
}

void PhysIKASystem::Render(void *pOptions, double _time, float _deltatime) {
    if (m_sensor_texID == 0) {
        glCreateTextures(GL_TEXTURE_2D, 1, &m_sensor_texID);
        glTextureStorage2D(m_sensor_texID, 1, GL_RGB8, 400, 300);
    }

    if (!m_config.IsServer() && m_sensor_view) {
        /*if (m_sensor_dirty) {
            m_sensor_dirty = false;
            glTextureSubImage2D(m_sensor_texID, 0, 0, 0, 400, 300, GL_RGB, GL_UNSIGNED_BYTE, m_sensor_data);
        }
        glDisable(GL_DEPTH_TEST);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_sensor_texID);
        OpenGLUtils::PushProgram(m_sensor_shader.get());
        glBindVertexArray(m_sensor_vertex_vao.get());
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        OpenGLUtils::PopProgram();*/
    }

    if (m_drawDebugGizmos) {
        for (auto &[_, height]: m_enttToHeightField) {
            DrawHeightFieldGizmos(height.get());
        }
        auto world = ViWoROOT::World();
        for (auto e: world->view<SandSimulationRegionComponent>()) {
            world->get<SandSimulationRegionComponent>(e).DrawGizmos();
        }

        PolygonRegion::DrawGizmos();
    }
}

void PhysIKASystem::DrawDebugGui() {
    if (ImGui::CollapsingHeader("Physika System")) {
        ImGui::Checkbox("Enable Simulation", &engine_work_);
        ImGui::Checkbox("Draw Debug Gizmos", &m_drawDebugGizmos);
        auto world = ViWoROOT::World();
        for (auto &car: m_enttToPBDCar) {
            auto car_ent_id = car.first;
            auto pbd_car = car.second;
            ImGui::Text("Car %s", world->GetName(static_cast<VPE::Entity>(car_ent_id)));
        }
        for (auto &car: m_enttToMWCar) {
            auto car_ent_id = car.first;
            auto mw_car = car.second;
            ImGui::Text("Car %s", world->GetName(static_cast<VPE::Entity>(car_ent_id)));
        }

        int id = 0;
        for (auto e: world->view<SandSimulationRegionComponent>()) {
            auto &region = world->get<SandSimulationRegionComponent>(e);
            if (ImGui::TreeNode((void *)(id++), "Sand Simulation Region <%s>", world->GetName(e).c_str())) {
                region.DrawDebugGui();
                ImGui::TreePop();
            }
        }
    }
}

void PhysIKASystem::UpdateHeightField() {
    auto world = ViWoROOT::World();

    /**
     * 因为使用Physika时只有一个Node，因此只有一块高度图
     * 所以 应该遍历场景内依赖地形的实体 以判断是否更新高度图
     */

    for (auto &car_height: m_enttToHeightField) {
        auto carent = car_height.first;
        auto height = car_height.second;

        if (carent == VPE::EntityNull)
            continue;

        auto car_pos = world->GetGeoPosition(carent);

        // 64 意味着 距离边缘还有2个地形块的时候
        // 这一块可以改成自适应，计算车身的最长轴方向的一半，只要车身重心到高度图的边缘 大于该值即可，否则更新
        if (!height->CoverRange(car_pos, car_pos, 64)) {
            auto hfiNde = m_enttToPhyNode[car_height.first];
            auto terrain = ViWoROOT::GetTerrainInterface();
            if (terrain) {
                //Noted by WR 更换高程接口
                //terrain->GetDemData(car_pos, height.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());
                SandSimulationRegionComponent::GetTerrHeight(car_pos, height.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());
            }

            height->SetPhysicalWorldCenterElevation(m_viwoOriginGeoPosition.z);

            PhysIKA::Vector3f heightMapCenterPosInPhysiKA;
            TransfromViwoCoordToPhysiKACoord(heightMapCenterPosInPhysiKA, height->CenterPosInViwoGlobalCoord());
            hfiNde->getHeightField().set<DeviceType::CPU>(height->Data(),
                                                          height->GridSize(), height->GridSize(), height->GridSize(),
                                                          height->DeltaX(), height->DeltaY(),
                                                          heightMapCenterPosInPhysiKA[0], heightMapCenterPosInPhysiKA[1], heightMapCenterPosInPhysiKA[2]);
        }
    }
}

namespace {
CarGroundConditionFlags GetCarGroundConditionFlags(VPE::Entity car) {
    auto world = ViWoROOT::World();

    for (auto &region_entity: world->view<PolygonRegion, CarGroundCondition>()) {
        auto &condition = world->get<CarGroundCondition>(region_entity);
        auto &region = world->get<PolygonRegion>(region_entity);

        auto pos_ws = world->GetPosition(car);
        auto world_to_local = world->GetTransform(region_entity).Inverse();
        auto pos_local = region.WorldToLocalPosition(world_to_local, pos_ws);
        if (region.Contains(pos_local)) {
            return condition.conditions;
        }
    }

    return {};
}

void ApplyCarEntityGroundConditions(VPE::Entity entity, CarGroundConditionFlags conditions) {
    auto world = ViWoROOT::World();
    auto car = world->try_get<ViWoCar>(entity);
    if (car == nullptr) {
        return;
    }

    auto particle = world->GetPart(entity, car->particle_tail_id);
    if (world->valid(particle)) {
        bool hide_particles = conditions & CAR_GROUND_CONDITION_NO_TRAIL_PARTICLES_BIT;
        world->SetVisible(particle, !hide_particles);
    }
}

void ApplyPBDCarGroundConditions(PhysIKA::PBDCar *car, CarGroundConditionFlags conditions) {
    if (conditions & CAR_GROUND_CONDITION_SLIPPERY_BIT) {
        car->set_SlipperyRate(1.5f);
    } else {
        car->set_SlipperyRate(1.0f);
    }
    if (conditions & CAR_GROUND_CONDITION_SLOW_DOWN_BIT) {
        car->set_linearVelocityDamping(0.7f);
    } else {
        car->set_linearVelocityDamping(0.99f);
    }
}
}  // namespace

void PhysIKASystem::UpdateCarGroundConditions() {
    for (auto &[entity, car]: m_enttToPBDCar) {
        auto conditions = GetCarGroundConditionFlags(entity);
        ApplyCarEntityGroundConditions(entity, conditions);
        ApplyPBDCarGroundConditions(car.get(), conditions);
    }
    for (auto &[entity, car]: m_enttToMWCar) {
        auto conditions = GetCarGroundConditionFlags(entity);
        ApplyCarEntityGroundConditions(entity, conditions);
    }
}

void PhysIKASystem::UpdateViwoCar() {
    auto world = ViWoROOT::World();

    for (auto &car: m_enttToPBDCar) {
        auto car_ent = car.first;
        auto pbd_car = car.second;
        if (pbd_car) {
            UpdateEntity(car_ent, pbd_car->m_chassis->getGlobalR(), pbd_car->m_chassis->getGlobalQ());
            auto viwo_car = world->try_get<ViWoCar>(car_ent);

            // 处理粒子系统的烟雾效果
            if (viwo_car->particle_tail_id) {
                float speed_ = pbd_car->m_chassis->getLinearVelocity().norm();
                float speed = speed_ > 1 ? 1 : speed_;
                auto particle = world->GetPart(car_ent, viwo_car->particle_tail_id);
                UpdateEntity(particle, VPE::vec3(0.65, 0.81, 0.5) * speed);
            }

            for (auto i = 0; i < 4; i++) {
                auto wheel_part_id = viwo_car->QueryFourWheelPhysikaCarWheelIDInViWoCar(i);
                auto wheel = world->GetPart(car_ent, wheel_part_id);
                UpdateEntity(wheel, pbd_car->m_wheels[i]->getGlobalR(), pbd_car->m_wheels[i]->getGlobalQ());
            }
        }
    }

    for (auto &car: m_enttToMWCar) {
        auto car_ent = car.first;
        auto pbd_car = car.second;
        if (pbd_car) {
            UpdateEntity(car_ent, pbd_car->m_chassis->getGlobalR(), pbd_car->m_chassis->getGlobalQ());
            auto viwo_car = world->try_get<ViWoCar>(car_ent);

            // 处理粒子系统的烟雾效果
            if (viwo_car->particle_tail_id) {
                float speed_ = pbd_car->m_chassis->getLinearVelocity().norm();
                float speed = speed_ > 1 ? 1 : speed_;
                auto particle = world->GetPart(car_ent, viwo_car->particle_tail_id);
                UpdateEntity(particle, VPE::vec3(0.65, 0.81, 0.5) * speed);
            }

            for (auto left_right = 0; left_right < 2; left_right++) {
                for (auto wheel_id = 0; wheel_id < 4; wheel_id++) {
                    auto wheel_part_id = viwo_car->QueryMultiWheelPhysikaCarWheelIDInViWoCar(left_right, wheel_id);
                    auto wheel = world->GetPart(car_ent, wheel_part_id);
                    UpdateEntity(wheel,
                                 pbd_car->m_wheels[left_right][wheel_id]->getGlobalR(),
                                 pbd_car->m_wheels[left_right][wheel_id]->getGlobalQ());
                }
            }
        }
    }
}

template <typename Car>
void ApplyCarControl(Car *car, ViWoCarMovementActionFlags actions, double delta_time) {
    if (actions & VIWO_CAR_MOVEMENT_ACTION_FORWARD) {
        car->forward(delta_time);
    }
    if (actions & VIWO_CAR_MOVEMENT_ACTION_BACKWARD) {
        car->backward(delta_time);
    }
    if (actions & VIWO_CAR_MOVEMENT_ACTION_LEFT) {
        car->goLeft(delta_time);
    }
    if (actions & VIWO_CAR_MOVEMENT_ACTION_RIGHT) {
        car->goRight(delta_time);
    }
}

template <typename CarGroups>
void ApplyCarControlToGroup(CarGroups &car_group, double delta_time) {
    auto world = ViWoROOT::World();
    for (auto &[entity, car]: car_group) {
        auto &car_conf = world->get<ViWoCar>(entity);
        ApplyCarControl(car.get(), car_conf.move_action_flags, delta_time);
        car_conf.move_action_flags = 0;
    }
}

void PhysIKASystem::UpdateCarControls() {
    ApplyCarControlToGroup(m_enttToPBDCar, m_timeDelta);
    ApplyCarControlToGroup(m_enttToMWCar, m_timeDelta);
}

void PhysIKASystem::initConfig() {
    m_config.Load();

    m_viwoOriginGeoPosition = m_config.GetPhysicalOrigin();
    m_timeDelta = m_config.GetPhysicalEngineTimeDelta();

    InitOriginGeoPoint(m_viwoOriginGeoPosition);

    {  // init udp network;
        m_linker = new VSVLink::UDP2SocketLink;
        if (m_config.IsServer()) {
            m_linker->GetUDPReceiver()->Bind(m_config.GetServerListenPort());
            m_linker->SetReceiveCallback([this](const VSVLink::FrameData &data) {
                recive_net_data_server(data);
            });

            m_linker->GetUDPSender()->SetDestination(m_config.GetMultiCastIP(), m_config.GetClientListenPort());
            m_linker->GetUDPSender()->SetMulticastLocalInterface(m_config.GetServerIP());
        } else {
            m_linker->GetUDPReceiver()->Bind(m_config.GetClientListenPort());
            m_linker->GetUDPReceiver()->JoinMulticastGroup(m_config.GetMultiCastIP());
            m_linker->SetReceiveCallback([this](const VSVLink::FrameData &data) {
                recive_net_data_client(data);
            });
            m_linker->GetUDPSender()->SetDestination(m_config.GetServerIP(), m_config.GetServerListenPort());
        }
        m_linker->Start();
    }
}

void PhysIKASystem::initRender() {
    VPE::RenderSystem::CRenderSystem *system = ViWoROOT::GetRenderSystem();
    VPE::RenderSystem::IRenderInterface *render = system ? system->GetCurrentRender() : nullptr;
    m_sandRenderable = std::make_unique<SandRenderable>();
    if (render) {
        render->AddRenderable(m_sandRenderable.get(), "PhyIKASandRenderable");
        render->InsertRenderable(9999, this, "PhySensor");
        render->AddAfter(this);
    }

    m_sensor_vertex_buf.alloc();
    m_sensor_vertex_vao.alloc();

    const float vertices[] = {
        -1.0f,
        1.0f,
        1.0f,
        1.0f,
        -1.0f,
        -1.0f,
        1.0f,
        -1.0f,
    };
    glBindVertexArray(m_sensor_vertex_vao.get());
    glBindBuffer(GL_ARRAY_BUFFER, m_sensor_vertex_buf.get());
    glBufferStorage(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)(0));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    /*m_sensor_shader = OpenGLUtils::CompileAndLinkShaderProgram(OpenGLUtils::ShaderConfig()
                                                                   .Vertex("network_sensor.vert")
                                                                   .Fragment("network_sensor.frag"));*/
}

void PhysIKASystem::initMem() {
    if (!m_sensor_data) {
        m_sensor_data = MemSimplePool<unsigned char>::GetInstance(m_config.GetSensorDataSize().x * m_config.GetSensorDataSize().y * 3)->Allocate();  // 3 for channel;
    }
}

void PhysIKASystem::cleanMem() {
    if (m_sensor_data) {
        MemSimplePool<unsigned char>::GetInstance(m_config.GetSensorDataSize().x * m_config.GetSensorDataSize().y * 3)->Free(m_sensor_data);
        m_sensor_data = 0;
    }
}

void PhysIKASystem::sendEnttToClient() {
    if (m_config.IsServer()) {
        m_linker->Send(m_send_to_client.GetBuffer());
        m_send_to_client.Reset();
    }
}

template <typename T>
void fillGrid2D(PhysIKA::HeightFieldGrid<T, T, DeviceType::CPU> &grid, const std::vector<int> &block, T value) {
    assert(block.size() >= 4);

    for (int i = block[0]; i < block[1]; ++i) {
        for (int j = block[2]; j < block[3]; ++j) {
            //grid[i*ny + j] = value;
            grid(i, j) = value;
        }
    }
}

template <typename T>
void fillGrid2D(PhysIKA::HeightFieldGrid<T, T, DeviceType::CPU> &grid, T value) {
    int nx = grid.Nx(), ny = grid.Ny();
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            grid(i, j) = value;
        }
    }
}

void PhysIKASystem::StartSimulation() {
    // Reset state
    m_pbdSolverNode.reset();
    m_multiDetectModule.reset();
    m_enttToPhyNode.clear();
    m_enttToHeightField.clear();
    m_enttToPBDCar.clear();
    m_enttToMWCar.clear();

    auto root = PhysIKA::SceneGraph::getInstance().createNewScene<PhysIKA::Node>();
    m_pbdSolverNode = std::make_shared<PhysIKA::PBDSolverNode>();
    m_pbdSolverNode->getSolver()->m_numSubstep = m_config.GetPhysicalSolverStep();
    m_pbdSolverNode->getSolver()->m_numContactSolveIter = m_config.GetPhysicalContactSolveIter();

    m_pbdSolverNode->getSolver()->setUseGPU(m_config.UseGPUForPhysicalEngine());
    m_pbdSolverNode->setDt(m_timeDelta);
    root->addChild(m_pbdSolverNode);

    m_multiDetectModule = m_pbdSolverNode->addCustomModule<PhysIKA::MultiHeightFieldRigidDetectionModule>("Multi-HF-Rigid Detection");
    m_multiDetectModule->rigidContactDetector = std::make_shared<PhysIKA::OrientedBodyDetector>();
    m_multiDetectModule->bindDetectionMethod(m_pbdSolverNode->getSolver().get());

    auto world = ViWoROOT::World();
    for (auto &e: world->view<ViWoCar>()) {
        auto &car = world->get<ViWoCar>(e);
        auto wheel_count = car.WheelCount();
        if (wheel_count == 4) {
            addFourWheelCar(e);
        } else if (wheel_count == 8) {
            addMultiWheelCar(e);
        }
        // Assign a random id to it, essential for net control
        if (world->GetID(e) == InvalidID<EntityID>()) {
            auto id = rand();
            while (world->GetEntity(id) != VPE::EntityNull) {
                id = rand();
            }
            world->AssignID(e, rand());
        }
    }

    for (auto e: world->view<SandSimulationRegionComponent>()) {
        class SyncNode : public PhysIKA::Node {
        public:
            std::vector<PhysIKA::RigidBody2_ptr> src_rigids{};
            std::vector<std::shared_ptr<PhysIKARigidBody>> proxies{};
            VPE::vec3 offset{};

            void advance(float dt) override {
                int count = min(src_rigids.size(), proxies.size());
                for (int i = 0; i < count; i++) {
                    auto &src = src_rigids[i];
                    auto &dst = proxies[i];
                    auto pos = src->getGlobalR();
                    auto rot = src->getGlobalQ();
                    Vec3 proxy_pos{
                        pos[0] + offset.x,
                        pos[1] + offset.y,
                        pos[2] + offset.z,
                    };
                    Quat proxy_rot{rot.x(), rot.y(), rot.z(), rot.w()};
                    dst->SetGlobalPositionRotation(proxy_pos, proxy_rot);
                }
            }
        };

        auto sync_node = std::make_shared<SyncNode>();

        auto &src_rigids = sync_node->src_rigids;
        auto &proxies = sync_node->proxies;

        std::vector<PhysIKARigidBodyCreateInfo> proxy_infos{};

        auto add_cube_rb = [&](const PhysIKA::RigidBody2_ptr &rb) {
            src_rigids.push_back(rb);

            PhysIKARigidBodyCreateInfo info{};
            info.mass = 1.0f;
            info.shape_path = VPE::AssetDir + "PhysIKA/chassis_cube.obj";
            info.sdf_path = VPE::AssetDir + "PhysIKA/chassis_cube.sdf";
            proxy_infos.push_back(info);
        };

        auto add_wheel_rb = [&](const PhysIKA::RigidBody2_ptr &rb) {
            src_rigids.push_back(rb);

            PhysIKARigidBodyCreateInfo info{};
            info.mass = 1.0f;
            info.scale = 3.0f;
            info.shape_path = VPE::AssetDir + "PhysIKA/wheel.obj";
            info.sdf_path = VPE::AssetDir + "PhysIKA/wheel.sdf";

            proxy_infos.push_back(info);
        };

        for (auto &[_, car]: m_enttToPBDCar) {
            add_cube_rb(car->getChassis());
            for (int w = 0; w < 4; w++) {
                add_wheel_rb(car->getWheels(w));
            }
        }

        for (auto &[_, car]: m_enttToMWCar) {
            add_cube_rb(car->getChassis());
            for (int i = 0; i < 2; i++) {
                for (int w = 0; w < 4; w++) {
                    add_wheel_rb(car->m_wheels[i][w]);
                }
            }
        }

        auto &region = world->get<SandSimulationRegionComponent>(e);
        region.StartSimulation(proxy_infos);

        auto sand_sim = region.data->sim_region;

        root->addChild(sand_sim->GetRoot());

        for (int i = 0; i < proxy_infos.size(); i++) {
            proxies.push_back(sand_sim->GetRigidBody(i));
        }

        sync_node->offset = -region.CenterInSimulationLocal();

        sand_sim->GetRoot()->addChild(sync_node);
    }

    auto &scene = PhysIKA::SceneGraph::getInstance();
    // Force reinitialize
    scene.invalid();
    scene.initialize();
}

void PhysIKASystem::addFourWheelCar(VPE::Entity entity) {
    auto pbd_car = addPBDFourWheelCar(entity, m_pbdSolverNode->getSolver());
    if (!pbd_car) {
        return;
    }
    m_pbdSolverNode->addChild(pbd_car);
}

std::shared_ptr<PhysIKA::PBDCar> PhysIKASystem::addPBDFourWheelCar(VPE::Entity entity, std::shared_ptr<PhysIKA::PBDSolver> solver) {
    auto world = ViWoROOT::World();
    const VPE::dvec3 &car_pos = world->GetGeoPosition(entity);

    const VPE::dvec3 pos_test = world->GetPosition(entity);        //test

    auto hfrDetectModule = applyHeightFieldDetectionModule();
    auto heightMap = std::make_shared<HeightField>(m_config.GetHeightMapSize());
    //Noted by WR 更换高程接口
    //ViWoROOT::GetTerrainInterface()->GetDemData(car_pos, heightMap.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());
    SandSimulationRegionComponent::GetTerrHeight(car_pos, heightMap.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());

    heightMap->SetPhysicalWorldCenterElevation(m_viwoOriginGeoPosition.z);
    PhysIKA::Vector3f heightMapCenterPosInPhysiKA;
    TransfromViwoCoordToPhysiKACoord(heightMapCenterPosInPhysiKA, heightMap->CenterPosInViwoGlobalCoord());
    hfrDetectModule->getHeightField().set<DeviceType::CPU>(heightMap->Data(), heightMap->GridSize(), heightMap->GridSize(), heightMap->GridSize(),
                                                           heightMap->DeltaX(), heightMap->DeltaY(), heightMapCenterPosInPhysiKA[0], heightMapCenterPosInPhysiKA[1], heightMapCenterPosInPhysiKA[2]);

    RenderNode **carRenderNode = world->try_get<RenderNode *>(entity);
    if (!carRenderNode || !*carRenderNode) {
        return nullptr;
    }

    auto viwo_car = world->try_get<VPE::ViWoCar>(entity);
    const VPE::vec3 &car_scalef = world->GetScale(entity);

    auto pbd_car = std::make_shared<PhysIKA::PBDCar>();
    pbd_car->m_rigidSolver = solver;

    PhysIKA::Vector3f pos;
    PhysIKA::Quaternion<float> rot;
    GetColliderTransform(pos, rot, entity, 0);
    pbd_car->carPosition = pos;
    pbd_car->carRotation = rot;

    pbd_car->wheelupDirection = Convert(viwo_car->up_dir);
    pbd_car->wheelRightDirection = Convert(viwo_car->right_dir);

    float wheelm = viwo_car->wheel_mass;
    PhysIKA::Vector3f wheelI = PhysIKA::RigidUtil::calculateCylinderLocalInertia(wheelm, viwo_car->wheel_inertia[0], viwo_car->wheel_inertia[1], viwo_car->wheel_inertia[2]);
    pbd_car->chassisMeshScale = Convert(car_scalef);

    std::string origin_path = (*carRenderNode)->GetModel()->GetLoadOption().modelpath;
    pbd_car->chassisFile = origin_path.replace(origin_path.find_last_of("."), 4, ".obj");
    pbd_car->chassisMeshTranslate = PhysIKA::Vector3f(0, 0, 0);
    pbd_car->chassisMass = viwo_car->mass;
    pbd_car->chassisInertia = PhysIKA::RigidUtil::calculateCubeLocalInertia(pbd_car->chassisMass, pbd_car->chassisMeshScale);
    pbd_car->steeringLowerBound = viwo_car->steering_lower;
    pbd_car->steeringUpperBound = viwo_car->steering_upper;
    pbd_car->steeringSpeed = viwo_car->steering_speed;
    pbd_car->forwardForceAcc = viwo_car->forward_force;
    pbd_car->maxVel = viwo_car->max_speed;
    pbd_car->suspensionLength = viwo_car->suspension_length;
    pbd_car->suspensionStrength = viwo_car->suspension_strength;

    /************************************************************************/
    /*                        这段是丑陋的碰撞配置代码.                      */
    /*              其中chassisCollisionGroup是车身碰撞体的ID                */
    /*              其中chassisCollisionMask是与之碰撞的物体ID               */
    /*              由于是位运算，所以只能是1,2,4,8,16,32,64...              */
    /************************************************************************/

    pbd_car->chassisCollisionGroup = viwo_car->chassis_collision_id;
    int collision_mask = 0;
    for (const auto c_id: viwo_car->chassis_collision_mask)
        collision_mask |= c_id;
    pbd_car->chassisCollisionMask = collision_mask;
    pbd_car->wheelCollisionGroup = viwo_car->wheel_collision_id;
    collision_mask = 0;
    for (const auto c_id: viwo_car->wheel_collision_mask)
        collision_mask |= c_id;
    pbd_car->wheelCollisionMask = collision_mask;

    pbd_car->linearDamping = viwo_car->linear_damping;
    pbd_car->angularDamping = viwo_car->angular_damping;

    for (int i = 0; i < 4; i++) {
        Entity wheel_entt = world->GetPart(entity, viwo_car->QueryFourWheelPhysikaCarWheelIDInViWoCar(i));
        if (wheel_entt == EntityNull) {
            return nullptr;
        }
        RenderNode **wheelRenderNode = world->try_get<RenderNode *>(wheel_entt);
        if (!wheelRenderNode || !*wheelRenderNode) {
            return nullptr;
        }
        const VPE::dvec3 &wheel_scalef = world->GetScale(wheel_entt);
        const VPE::dvec3 &pos = world->GetLocalPosition(wheel_entt);
        const VPE::dquat &rot = world->GetRotation(wheel_entt);
        pbd_car->wheelRelPosition[i] = PhysIKA::Vector3f(pos.x * wheel_scalef.x, pos.y * wheel_scalef.y, pos.z * wheel_scalef.z);
        pbd_car->wheelMeshScale[i] = PhysIKA::Vector3f(wheel_scalef.x, wheel_scalef.y, wheel_scalef.z);
        pbd_car->wheelMeshTranslate[i] = PhysIKA::Vector3f(0, 0, 0);

        // trick for wheel collider
        origin_path = (*wheelRenderNode)->GetModel()->GetLoadOption().modelpath;
        pbd_car->wheelFile[i] = origin_path.replace(origin_path.find_last_of("."), 4, ".obj");

        pbd_car->wheelRelRotation[i] = PhysIKA::Quaternion<float>(0, 0, 0, 1);
        pbd_car->wheelMass[i] = wheelm;
        pbd_car->wheelInertia[i] = wheelI;
    }
    pbd_car->build();
    pbd_car->setDt(m_timeDelta);

    makeCollider(pbd_car->getChassis()->getTopologyModule(), pbd_car->m_chassis);
    for (int i = 0; i < 4; ++i) {
        auto pwheel = pbd_car->m_wheels[i];
        makeCollider(pwheel->getTopologyModule(), pwheel);
    }

    m_enttToHeightField[entity] = heightMap;
    m_enttToPBDCar[entity] = pbd_car;
    m_enttToPhyNode[entity] = hfrDetectModule;
    m_all_cars.push_back(entity);

    for (auto i = 0; i < 4; i++) {
        hfrDetectModule->rigids.push_back(pbd_car->m_wheels[i]);
    }
    hfrDetectModule->rigids.push_back(pbd_car->m_chassis);

    return pbd_car;
}

void PhysIKASystem::addMultiWheelCar(VPE::Entity entity) {
    auto world = ViWoROOT::World();

    auto hfrDetectModule = applyHeightFieldDetectionModule();
    auto heightMap = std::make_shared<HeightField>(m_config.GetHeightMapSize());
    const VPE::dvec3 &car_pos = world->GetGeoPosition(entity);
    //ViWoROOT::GetTerrainInterface()->GetDemData(car_pos, heightMap.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());
    SandSimulationRegionComponent::GetTerrHeight(car_pos, heightMap.get(), m_config.GetHeightMapPrecisionLevelInTerrainQuadTree());
    heightMap->SetPhysicalWorldCenterElevation(m_viwoOriginGeoPosition.z);
    PhysIKA::Vector3f heightMapCenterPosInPhysiKA;
    TransfromViwoCoordToPhysiKACoord(heightMapCenterPosInPhysiKA, heightMap->CenterPosInViwoGlobalCoord());
    hfrDetectModule->getHeightField().set<DeviceType::CPU>(heightMap->Data(), heightMap->GridSize(), heightMap->GridSize(), heightMap->GridSize(),
                                                           heightMap->DeltaX(), heightMap->DeltaY(), heightMapCenterPosInPhysiKA[0], heightMapCenterPosInPhysiKA[1], heightMapCenterPosInPhysiKA[2]);

    RenderNode **carRenderNode = world->try_get<RenderNode *>(entity);
    if (!carRenderNode || !*carRenderNode) {
        return;
    }

    auto viwo_car = world->try_get<VPE::ViWoCar>(entity);
    const VPE::vec3 &car_scalef = world->GetScale(entity);

    auto mul_whl_car = std::make_shared<PhysIKA::MultiWheelCar<4>>();
    mul_whl_car->m_rigidSolver = m_pbdSolverNode->getSolver();

    PhysIKA::Vector3f pos;
    PhysIKA::Quaternion<float> rot;
    GetColliderTransform(pos, rot, entity, 0);
    mul_whl_car->carPosition = pos;
    mul_whl_car->carRotation = rot;

    mul_whl_car->upDirection = Convert(viwo_car->up_dir);
    mul_whl_car->rightDirection = Convert(viwo_car->right_dir);

    std::string origin_path = (*carRenderNode)->GetModel()->GetLoadOption().modelpath;
    std::string path_ = origin_path.replace(origin_path.find_last_of("."), 4, ".obj");
    mul_whl_car->chassisFile = path_;
    mul_whl_car->chassisMeshScale = Convert(car_scalef);
    mul_whl_car->chassisMeshTranslate = PhysIKA::Vector3f(0, 0, 0);
    mul_whl_car->chassisMass = viwo_car->mass;
    mul_whl_car->chassisInertia = PhysIKA::RigidUtil::calculateCubeLocalInertia(mul_whl_car->chassisMass, mul_whl_car->chassisMeshScale);

    for (auto left_right = 0; left_right < 2; left_right++) {
        for (auto wheel_id = 0; wheel_id < viwo_car->WheelCount() / 2; wheel_id++) {
            Entity wheel_entt = world->GetPart(entity, viwo_car->QueryMultiWheelPhysikaCarWheelIDInViWoCar(left_right, wheel_id));
            if (wheel_entt == EntityNull) {
                return;
            }
            RenderNode **wheelRenderNode = world->try_get<RenderNode *>(wheel_entt);
            if (!wheelRenderNode || !*wheelRenderNode) {
                return;
            }

            const VPE::dvec3 &wheel_scalef = world->GetScale(wheel_entt);
            const VPE::dvec3 &pos = world->GetLocalPosition(wheel_entt);
            const VPE::dquat &rot = world->GetRotation(wheel_entt);

            const auto &inertia = viwo_car->wheel_inertia;
            PhysIKA::Vector3f wheelI = PhysIKA::RigidUtil::calculateCylinderLocalInertia(viwo_car->wheel_mass, inertia.x, inertia.y, inertia.z);
            mul_whl_car->wheelMass[left_right][wheel_id] = viwo_car->wheel_mass;
            mul_whl_car->wheelInertia[left_right][wheel_id] = wheelI;
            mul_whl_car->wheelRelPos[left_right][wheel_id] = PhysIKA::Vector3f(pos.x * wheel_scalef.x, pos.y * wheel_scalef.y, pos.z * wheel_scalef.z);
            mul_whl_car->wheelRelRot[left_right][wheel_id] = PhysIKA::Quaternion<float>(0, 0, 0, 1);
            mul_whl_car->wheelMeshScale[left_right][wheel_id] = PhysIKA::Vector3f(wheel_scalef.x, wheel_scalef.y, wheel_scalef.z);
            mul_whl_car->wheelMeshTranslate[left_right][wheel_id] = PhysIKA::Vector3f(0, 0, 0);

            // trick for wheel collider
            origin_path = (*wheelRenderNode)->GetModel()->GetLoadOption().modelpath;
            mul_whl_car->wheelFile[left_right][wheel_id] = origin_path.replace(origin_path.find_last_of("."), 4, ".obj");
        }
    }

    mul_whl_car->forwardForceAcc = viwo_car->forward_force;
    mul_whl_car->maxVel = viwo_car->max_speed;

    /************************************************************************/
    /*                        这段是丑陋的碰撞配置代码                       */
    /*              其中wheelCollisionGroup是车轮碰撞体的ID                  */
    /*              其中wheelCollisionMask是与之碰撞物体ID                   */
    /************************************************************************/

    mul_whl_car->chassisCollisionGroup = viwo_car->chassis_collision_id;
    int collision_mask = 0;
    for (const auto c_id: viwo_car->chassis_collision_mask)
        collision_mask |= c_id;
    mul_whl_car->chassisCollisionMask = collision_mask;
    mul_whl_car->wheelCollisionGroup = viwo_car->wheel_collision_id;
    collision_mask = 0;
    for (const auto c_id: viwo_car->wheel_collision_mask)
        collision_mask |= c_id;
    mul_whl_car->wheelCollisionMask = collision_mask;

    mul_whl_car->linearDamping = viwo_car->linear_damping;
    mul_whl_car->angularDamping = viwo_car->angular_damping;

    mul_whl_car->suspensionLength = viwo_car->suspension_length;
    mul_whl_car->suspensionStrength = viwo_car->suspension_strength;

    mul_whl_car->build();
    mul_whl_car->setDt(m_timeDelta);

    hfrDetectModule->rigids.push_back(mul_whl_car->m_chassis);
    for (auto left_right = 0; left_right < 2; left_right++) {
        for (auto wheel_id = 0; wheel_id < viwo_car->WheelCount() / 2; wheel_id++) {
            hfrDetectModule->rigids.push_back(mul_whl_car->m_wheels[left_right][wheel_id]);
        }
    }
    m_pbdSolverNode->addChild(mul_whl_car);

    makeCollider(mul_whl_car->getChassis()->getTopologyModule(), mul_whl_car->m_chassis);
    for (int lr = 0; lr < 2; ++lr) {
        for (int i = 0; i < viwo_car->WheelCount() / 2; ++i) {
            auto pwheel = mul_whl_car->m_wheels[lr][i];
            makeCollider(pwheel->getTopologyModule(), pwheel);
        }
    }

    m_enttToHeightField[entity] = heightMap;
    m_enttToMWCar[entity] = mul_whl_car;
    m_enttToPhyNode[entity] = hfrDetectModule;
    m_all_cars.push_back(entity);
}

void PhysIKASystem::makeCollider(const std::shared_ptr<PhysIKA::TopologyModule> &topo, const PhysIKA::RigidBody2_ptr &obj) {
    auto pointset = TypeInfo::cast<PhysIKA::PointSet<PhysIKA::DataType3f>>(topo);
    if (pointset) {
        std::shared_ptr<PhysIKA::TOrientedBox3D<float>> pobb = std::make_shared<PhysIKA::TOrientedBox3D<float>>();
        pobb->u = PhysIKA::Vector3f(1, 0, 0);
        pobb->v = PhysIKA::Vector3f(0, 1, 0);
        pobb->w = PhysIKA::Vector3f(0, 0, 1);
        this->computeAABB(pointset, pobb->center, pobb->extent);

        auto pdetector = m_multiDetectModule->rigidContactDetector;
        pdetector->addCollidableObject(obj, pobb);
    }
}

void PhysIKASystem::recive_net_data_client(const VSVLink::FrameData &data) {
    PhyCommandDecoder decoder(data.buffer);

    PHYSIKA_NET_DATA_TYPE cmd_type = PHY_NET_DATA_TYPE_START;
    VSVLink::ConstBuffer cmd_buffer;
    auto world = ViWoROOT::World();

    while (std::tie(cmd_type, cmd_buffer) = decoder.NextCommand(), cmd_type != PHY_NET_DATA_TYPE_START && cmd_buffer.size != 0) {
        switch (cmd_type) {
        case ENTITY_POS: {
            auto *cmd = reinterpret_cast<const PhysiKaEntityPosCmd *>(cmd_buffer.data);
            auto entt_id = cmd->entity_id;
            VPE::dvec3 pos = {cmd->posx, cmd->posy, cmd->posz};
            VPE::dquat qua = {cmd->quaw, cmd->quax, cmd->quay, cmd->quaz};
            UpdateEntity(world->GetEntity(entt_id), pos, qua);
        } break;
        case ENTITY_SCALE: {
            auto *cmd = reinterpret_cast<const PhysiKaEntityScaleCmd *>(cmd_buffer.data);
            UpdateEntity(world->GetEntity(cmd->entity_id), cmd->scale);
        } break;
        case SENSOR_DATA:
            handle_sensor_data(cmd_buffer.data);
            break;
        }
    }
}

void PhysIKASystem::recive_net_data_server(const VSVLink::FrameData &data) {
    if (!EngineWorking()) {
        return;
    }

    PhyCommandDecoder decoder(data.buffer);

    PHYSIKA_NET_DATA_TYPE cmd_type = PHY_NET_DATA_TYPE_START;
    VSVLink::ConstBuffer cmd_buffer;

    while (std::tie(cmd_type, cmd_buffer) = decoder.NextCommand(), cmd_type != PHY_NET_DATA_TYPE_START && cmd_buffer.size != 0) {
        switch (cmd_type) {
        case KEY_CONTROL: {
            const auto *cmd = reinterpret_cast<const PhysiKaKeyboardCmd *>(cmd_buffer.data);
            handle_client_control(cmd);
            break;
        }
        case ENTITY_SENSOR: {
            const auto *cmd = reinterpret_cast<const PhysiKaEntitySensorCmd *>(cmd_buffer.data);
            handle_sensor_control(cmd);
            break;
        }
        }
    }
}

void PhysIKASystem::UpdateEntity(VPE::Entity entt, const PhysIKA::Vector3f &posw, const PhysIKA::Quaternionf &rotw) {
    auto world = ViWoROOT::World();
    if (entt == VPE::EntityNull) {
        return;
    }

    VPE::dvec3 vposw;  //= Convert(pos) + m_viwoOriginGlobalPosition;
    VPE::dquat vquaw;  // = Convert(rot);
    TransfromPhysiKACoordToViwoCoord(vposw, vquaw, posw, rotw);

    if (VPE::all(VPE::isnan(vposw)) || VPE::all(VPE::isnan(vquaw))) {
        return;
    }

    Transform transw;
    transw.SetPosition(vposw);
    transw.SetRotation(vquaw);
    transw.SetScale(world->GetTransform(entt).GetScale());
    world->SetTransform(entt, transw);

    if (m_config.IsServer()) {
        PhysiKaEntityPosCmd cmd;
        cmd.entity_id = world->GetID(entt);
        std::tie(cmd.posx, cmd.posy, cmd.posz) = std::tie(vposw.x, vposw.y, vposw.z);
        std::tie(cmd.quax, cmd.quay, cmd.quaz, cmd.quaw) = std::tie(vquaw.x, vquaw.y, vquaw.z, vquaw.w);
        VSVLink::ConstBuffer sub_buf(&cmd, sizeof(cmd));
        m_send_to_client.Append(sub_buf);
    }
}

void PhysIKASystem::UpdateEntity(VPE::Entity entt, const VPE::dvec3 &pos, const VPE::dquat &qua) {
    auto world = ViWoROOT::World();
    if (entt == VPE::EntityNull) {
        return;
    }
    Transform transw;
    transw.SetPosition(pos);
    transw.SetRotation(qua);
    transw.SetScale(world->GetTransform(entt).GetScale());
    world->SetTransform(entt, transw);

    if (m_track_moving_car && entt == m_all_cars[m_control_id])
        camera_track_moving_vehicle(pos);
}

void PhysIKASystem::UpdateEntity(VPE::Entity entity, const VPE::vec3 &scale) {
    auto world = ViWoROOT::World();
    if (entity != VPE::EntityNull) {
        world->SetScale(entity, scale);
    }
    if (m_config.IsServer()) {
        PhysiKaEntityScaleCmd cmd;
        cmd.entity_id = world->GetID(entity);
        cmd.scale = scale;
        VSVLink::ConstBuffer sub_buf(&cmd, sizeof(cmd));
        m_send_to_client.Append(sub_buf);
    }
}

inline std::shared_ptr<PhysIKA::HeightFieldRigidDetectionModule> PhysIKASystem::applyHeightFieldDetectionModule() {
    auto hfrigidDetectModule = std::make_shared<PhysIKA::HeightFieldRigidDetectionModule>();
    m_multiDetectModule->heightfieldRigidDetectors.push_back(hfrigidDetectModule);
    return hfrigidDetectModule;
}

void PhysIKASystem::camera_track_moving_vehicle(const VPE::dvec3 &track_pos) {
    //     镜头跟踪车辆
    auto cam = ViWoROOT::GetCameraManager()->GetCurrCamera();
    VPE::dvec3 campos = cam->getPosition();

    VPE::dvec3 carposgeo;
    g_coord.GlobalCoord2LongLat(track_pos, carposgeo);

    VPE::dvec3 dirdir = VPE::normalize(campos - track_pos);
    campos = track_pos + dirdir * dis_X * dis_Factor;

    VPE::dvec3 camposgeo;
    g_coord.GlobalCoord2LongLat(campos, camposgeo);
    camposgeo.z = carposgeo.z + dis_Y * dis_Factor;
    g_coord.LongLat2GlobalCoord(camposgeo, campos);

    VPE::dvec3 right = VPE::cross(dirdir, VPE::normalize(campos));
    VPE::dvec3 camup = VPE::cross(VPE::normalize(right), VPE::normalize(dirdir));

    cam->LookAt(campos, track_pos, camup);
}

void PhysIKASystem::vehicle_sensor() {
    auto e = m_all_cars[m_control_id];
    PhysiKaEntitySensorCmd cmd;
    cmd.entity_id = ViWoROOT::World()->GetID(e);
    cmd.sensor_switch = m_sensor_view;

    VSVLink::ConstBuffer sub_buf(&cmd, sizeof(cmd));
    m_send_to_server.Append(sub_buf);
}

void PhysIKASystem::handle_client_control(const PhysiKaKeyboardCmd *cmd) {
    if (!cmd) {
        return;
    }
    auto world = ViWoROOT::World();
    auto ent = world->GetEntity(cmd->entity_id);
    char val = cmd->key_val;

    auto car = world->try_get<ViWoCar>(ent);
    if (car == nullptr) {
        return;
    }
    auto &actions = car->move_action_flags;
    switch (val) {
    case 38:
        actions |= VIWO_CAR_MOVEMENT_ACTION_FORWARD;
        break;
    case 40:
        actions |= VIWO_CAR_MOVEMENT_ACTION_BACKWARD;
        break;
    case 37:
        actions |= VIWO_CAR_MOVEMENT_ACTION_LEFT;
        break;
    case 39:
        actions |= VIWO_CAR_MOVEMENT_ACTION_RIGHT;
        break;
    }
}

void PhysIKASystem::handle_sensor_control(const PhysiKaEntitySensorCmd *cmd) {
    if (!cmd) {
        return;
    }
    auto entt_id = cmd->entity_id;

    bool sensor_switch = cmd->sensor_switch;
    if (!sensor_switch) {
        m_sensor_entt_id = 0;
        m_sensor_view = false;
    }

    m_sensor_view = true;
    m_sensor_entt_id = entt_id;
}

void PhysIKASystem::update_camera() {
    if (!m_sensor_view) {
        return;
    }
    auto world = ViWoROOT::World();
    auto entt = world->GetEntity(m_sensor_entt_id);
    if (entt == VPE::EntityNull) {
        return;
    }

    auto cam = ViWoROOT::GetCameraManager()->GetCurrCamera();
    // update cam with vehicle
    {
        auto pos = world->GetPosition(entt);
        auto mtx = world->GetMatrix(entt);

        dvec4 for_dir(0, 0, -1, 0), up_dir(0, 1, 0, 0);
        auto dir = mtx * for_dir;
        auto up = mtx * up_dir;

        pos += up.xyz * 2.0;

        cam->LookAt(pos, pos + dir.xyz * 100.0, up.xyz);
    }

    static const int sensor_send_limit = monitor_frame_rate / m_config.GetSensorFrameRate();
    static int send_view = 0;
    if (send_view % sensor_send_limit == 0) {
        auto framebuffer = ViWoROOT::GetRenderSystem()->GetCurrentRender()->GetManagedFramebuffer();
        auto colorId = framebuffer->GetWorkingColor();

        //GLint ViewPort[4];
        //glGetIntegerv(GL_VIEWPORT, ViewPort);
        const VPE::ivec4 &ViewPort = cam->getViewPort();
        GLsizei ColorChannel = 4;
        GLsizei bufferSize = ViewPort[2] * ViewPort[3] * sizeof(GLubyte) * ColorChannel;
        GLubyte *ImgData = MemSimplePool<GLubyte>::GetInstance(bufferSize)->Allocate();
        unsigned char *frameMem = MemSimplePool<GLubyte>::GetInstance(65536)->Allocate();  // 65536 for UDP Max Buffer Size. Detail @ https://zhuanlan.zhihu.com/p/92958754

        glGetTextureSubImage(colorId, 0, 0, 0, 0, ViewPort[2], ViewPort[3], 1, GL_RGBA, GL_UNSIGNED_BYTE, bufferSize, ImgData);

        FIBITMAP *image = FreeImage_ConvertFromRawBits(ImgData, ViewPort[2], ViewPort[3], ViewPort[2] * 4, 32, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK, FALSE);
        fipImage img;
        img = image;
        if (img.isValid()) {
            img.rescale(m_config.GetSensorDataSize().x, m_config.GetSensorDataSize().y, FILTER_BICUBIC);
            unsigned char *end_buf = NULL;
            INT64 end_len = 0;

            // Free end_buf manually
            fipMemoryIO mem;
            img.saveToMemory(FIF_JPEG, mem);
            mem.acquire(&end_buf, &end_len);

            PhysiKaEntitySensorViewCmd cmd;
            cmd.entity_id = m_sensor_entt_id;
            cmd.header.cmd_size += end_len;

            memset(frameMem, 0, sizeof(unsigned char) * 65536);
            memcpy(frameMem, &cmd, sizeof(cmd));
            memcpy(frameMem + sizeof(cmd), end_buf, end_len);

            VSVLink::ConstBuffer buf(frameMem, cmd.header.cmd_size);
            m_send_to_client.Append(buf);
        }
        MemSimplePool<GLubyte>::GetInstance(bufferSize)->Free(ImgData);
        MemSimplePool<GLubyte>::GetInstance(65536)->Free(frameMem);
    }
    send_view++;
    send_view %= sensor_send_limit;
}

void PhysIKASystem::handle_sensor_data(const std::byte *cmd_data) {
    auto *cmd = reinterpret_cast<const PhysiKaEntitySensorViewCmd *>(cmd_data);
    auto ent_id = cmd->entity_id;
    auto img_size = cmd->header.cmd_size - sizeof(PhysiKaEntitySensorViewCmd);

    auto world = ViWoROOT::World();
    auto ent = world->GetEntity(ent_id);
    if (ent_id != world->GetID(m_all_cars[m_control_id])) {
        m_sensor_view = false;
        m_sensor_entt_id = 0;
        return;
    }
    if (ent == VPE::EntityNull) {
        return;
    }
    fipMemoryIO mem((uint8_t *)cmd_data + sizeof(PhysiKaEntitySensorViewCmd), img_size);
    fipImage img;
    img.loadFromMemory(mem);
    if (img.isValid()) {
        //img.Save("D:/test.jpg", CXIMAGE_FORMAT_JPG);
        memcpy((void *)m_sensor_data, img.accessPixels(), m_config.GetSensorDataSize().x * m_config.GetSensorDataSize().y * 3);
        m_sensor_dirty = true;
    }
}

PhysIKASystemBase *CreatePhysIKASystem() {
    return new PhysIKASystem();
}
}  // namespace VPE