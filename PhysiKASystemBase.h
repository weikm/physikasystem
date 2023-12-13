#pragma once
#include "PhysicsSystem.h"
#include "Event/EventHandler.hpp"
#include "Event/KeyBoardEvent.h"

#include "Core/DataTypes.h"
#include "Framework/Topology/PointSet.h"

#include "Renderable.h"

namespace VPE {
class CAxisAlignedBox;
class RigidBody;
class Collider;
class Ray;
class HeightField;

class PhysIKASystemBase : public VPE::PhysicsSystem, public EventHandler<KeyBoardEvent>, public RenderSystem::IRenderHook_After, public RenderSystem::IRenderable {
public:
    PhysIKASystemBase();
    virtual ~PhysIKASystemBase();

public:
    void EnableDebugDraw(bool enable) override{};

    virtual void OnVehicleAdded(entt::registry &, VPE::Entity entity){};
    virtual void OnVehicleRemoved(entt::registry &, VPE::Entity entity){};
    void OnEntityTransformed(VPE::Entity entity, unsigned int trans_flag) override{};
    void RayTest(const VPE::Ray &ray, double range, RaytestResult &results, bool onehit, VPE::LayerMask mask) override{};
    void contactTest(const VPE::dvec3 &from, const VPE::dvec3 &to, const VPE::dvec3 &boxHalfExtents, RaytestResult &results, LayerMask mask) override;

    void InitOriginGeoPoint(const VPE::dvec3 &origin_geo) override;

    void PostRender(void *pOptions, double _time, float _deltatime){};
    void Render(void *pOptions, double _time, float _deltatime){};
    VPE::dmat4 SimulationRegionLocalToWorld();

protected:
    virtual void initConfig() {}

    PhysIKA::Vector3f Convert(const VPE::fvec3 &);
    PhysIKA::Vector3d Convert(const VPE::dvec3 &);

    PhysIKA::Quaternionf Convert(const VPE::fquat &);
    PhysIKA::Quaterniond Convert(const VPE::dquat &);

    VPE::fvec3 Convert(const PhysIKA::Vector3f &);
    VPE::dvec3 Convert(const PhysIKA::Vector3d &);

    VPE::dquat Convert(const PhysIKA::Quaterniond &);
    VPE::fquat Convert(const PhysIKA::Quaternionf &);

protected:
    void GetColliderTransform(PhysIKA::Vector3f &pos, PhysIKA::Quaternion<float> &rot, VPE::Entity entity, const VPE::Collider *collider);

    void TransfromViwoCoordToPhysiKACoord(PhysIKA::Vector3f &phPos, PhysIKA::Quaternionf &phRot, const VPE::dvec3 &viPos, const VPE::dquat &viRot);

    void TransfromViwoCoordToPhysiKACoord(PhysIKA::Vector3f &phPos, const VPE::dvec3 &viPos);

    void TransfromPhysiKACoordToViwoCoord(VPE::dvec3 &viPos, VPE::dquat &viRot, const PhysIKA::Vector3f &phPos, const PhysIKA::Quaternionf &phRot);

    void makeTransformRot(const VPE::dvec3 &geo_pos);

    void makeTransformRot(double lon, double lat, double height);

    void computeAABB(std::shared_ptr<PhysIKA::PointSet<PhysIKA::DataType3f>> points, PhysIKA::Vector3f &center, PhysIKA::Vector3f &halfSize);

protected:
    float m_timeDelta;

    VPE::dquat m_globalRot;  // Simulation frame rotation in global coordinate.
    VPE::dvec3 m_viwoOriginGlobalPosition;

    std::vector<VPE::Entity> m_all_cars;
    std::size_t m_control_id;
};

PhysIKASystemBase *CreatePhysIKASystem();
}  // namespace VPE