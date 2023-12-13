#include "PhysicsSystem.h"
#include "PhysiKASystemBase.h"

#include "ViWoRoot.h"
#include "World.h"
#include "Coordination.h"
#include "Config/CoreConfig.h"
#include "StringUtils.h"
#include "ViwoCar.h"

#include "Collider.h"

namespace VPE {
PhysIKASystemBase::PhysIKASystemBase() {
    gravity_ = -9.8;
    auto world = ViWoROOT::World();
    world->on_construct<ViWoCar>().connect<&PhysIKASystemBase::OnVehicleAdded>(*this);
    world->on_destroy<ViWoCar>().connect<&PhysIKASystemBase::OnVehicleRemoved>(*this);
}

PhysIKASystemBase::~PhysIKASystemBase() {
    auto world = ViWoROOT::World();
    world->on_construct<ViWoCar>().disconnect<&PhysIKASystemBase::OnVehicleAdded>(*this);
    world->on_destroy<ViWoCar>().disconnect<&PhysIKASystemBase::OnVehicleRemoved>(*this);
}

void PhysIKASystemBase::GetColliderTransform(PhysIKA::Vector3f &pos, PhysIKA::Quaternion<float> &rot, VPE::Entity entity, const VPE::Collider *collider) {
    VPE::Transform trans = ViWoROOT::World()->GetTransform(entity);
    TransfromViwoCoordToPhysiKACoord(pos, rot, trans.GetPosition(), trans.GetRotation());
}

void PhysIKASystemBase::TransfromViwoCoordToPhysiKACoord(PhysIKA::Vector3f &phPos, const VPE::dvec3 &viPos) {
    VPE::dquat invFrameRot = VPE::inverse(m_globalRot);

    VPE::dvec3 pos = VPE::rotate(invFrameRot, viPos - m_viwoOriginGlobalPosition);
    PhysIKA::Vector3d phPosd = Convert(pos);
    phPos = PhysIKA::Vector3f(phPosd[0], phPosd[1], phPosd[2]);
}

void PhysIKASystemBase::TransfromViwoCoordToPhysiKACoord(PhysIKA::Vector3f &phPos, PhysIKA::Quaternionf &phRot, const VPE::dvec3 &viPos, const VPE::dquat &viRot) {
    TransfromViwoCoordToPhysiKACoord(phPos, viPos);
    VPE::fquat rot = VPE::inverse(m_globalRot) * viRot;
    phRot = Convert(rot);
}

void PhysIKASystemBase::TransfromPhysiKACoordToViwoCoord(VPE::dvec3 &viPos, VPE::dquat &viRot, const PhysIKA::Vector3f &phPos, const PhysIKA::Quaternionf &phRot) {
    PhysIKA::Vector3d phPosd(phPos[0], phPos[1], phPos[2]);
    PhysIKA::Quaterniond phRotd(phRot[0], phRot[1], phRot[2], phRot[3]);

    PhysIKA::Quaterniond gRot = Convert(m_globalRot);
    PhysIKA::Vector3d pos = gRot.rotate(phPosd);
    viPos = m_viwoOriginGlobalPosition + Convert(pos);
    viRot = m_globalRot * Convert(phRotd);
}

void PhysIKASystemBase::InitOriginGeoPoint(const VPE::dvec3 &origin_geo) {
    makeTransformRot(origin_geo);
    g_coord.LongLat2GlobalCoord(origin_geo, m_viwoOriginGlobalPosition);
}

VPE::dmat4 PhysIKASystemBase::SimulationRegionLocalToWorld() {
    return VPE::translate(m_viwoOriginGlobalPosition) * VPE::mat4_cast<double>(m_globalRot);
}

void PhysIKASystemBase::makeTransformRot(const VPE::dvec3 &geo_pos) {
    makeTransformRot(geo_pos.x, geo_pos.y, geo_pos.z);
}

void PhysIKASystemBase::makeTransformRot(double lon, double lat, double height) {

    Transform trans;
    VPE::dvec3 dirLon, dirLat;

    VPE::dvec3 p0;
    g_coord.LongLat2GlobalCoord(lon, lat, height, p0);

    VPE::dvec3 pLon;
    g_coord.LongLat2GlobalCoord(lon + 0.00001, lat, height, pLon);
    dirLon = VPE::normalize(pLon - p0);

    VPE::dvec3 pLat;
    g_coord.LongLat2GlobalCoord(lon, lat - 0.00001, height, pLat);
    dirLat = VPE::normalize(pLat - p0);

    VPE::dvec3 dirup = VPE::normalize(VPE::cross(dirLat, dirLon));
    dirLat = VPE::normalize(VPE::cross(dirLon, dirup));

    VPE::dvec3 gDirx(1.0, 0.0, 0.0);
    VPE::dvec3 gDiry(0.0, 1.0, 0.0);

    VPE::dquat rotx(gDirx, dirLon);
    gDiry = VPE::rotate(rotx, gDiry);
    VPE::dquat roty(gDiry, dirup);

    m_globalRot = roty * rotx;
}

PhysIKA::Vector3d PhysIKASystemBase::Convert(const VPE::dvec3 &vec) {
    return PhysIKA::Vector3d(vec.x, vec.y, vec.z);
}
PhysIKA::Vector3f PhysIKASystemBase::Convert(const VPE::fvec3 &vec) {
    return PhysIKA::Vector3f(vec.x, vec.y, vec.z);
}
PhysIKA::Quaternionf PhysIKASystemBase::Convert(const VPE::fquat &quat) {
    return PhysIKA::Quaternionf(quat.x, quat.y, quat.z, quat.w);
}
PhysIKA::Quaterniond PhysIKASystemBase::Convert(const VPE::dquat &quat) {
    return PhysIKA::Quaterniond(quat.x, quat.y, quat.z, quat.w);
}
VPE::dvec3 PhysIKASystemBase::Convert(const PhysIKA::Vector3d &phvec3) {
    return VPE::dvec3(phvec3[0], phvec3[1], phvec3[2]);
}
VPE::fvec3 PhysIKASystemBase::Convert(const PhysIKA::Vector3f &phvec3) {
    return VPE::dvec3(phvec3[0], phvec3[1], phvec3[2]);
}
VPE::fquat PhysIKASystemBase::Convert(const PhysIKA::Quaternionf &qua) {
    return VPE::dquat(qua.w(), qua.x(), qua.y(), qua.z());
}
VPE::dquat PhysIKASystemBase::Convert(const PhysIKA::Quaterniond &qua) {
    return VPE::dquat(qua.w(), qua.x(), qua.y(), qua.z());
}

void PhysIKASystemBase::computeAABB(std::shared_ptr<PhysIKA::PointSet<PhysIKA::DataType3f>> points, PhysIKA::Vector3f &center, PhysIKA::Vector3f &halfSize) {
    int nPoints = points->getPointSize();
    if (nPoints <= 0)
        return;

    auto &pointArr = points->getPoints();
    PhysIKA::HostArray<PhysIKA::Vector3f> hpoints;
    hpoints.resize(nPoints);
    PhysIKA::Function1Pt::copy(hpoints, pointArr);

    PhysIKA::Vector3f pmin = hpoints[0];
    PhysIKA::Vector3f pmax = hpoints[0];
    for (int i = 1; i < nPoints; ++i) {
        PhysIKA::Vector3f curp = hpoints[i];
        pmin[0] = min(pmin[0], curp[0]);
        pmin[1] = min(pmin[1], curp[1]);
        pmin[2] = min(pmin[2], curp[2]);
        pmax[0] = max(pmax[0], curp[0]);
        pmax[1] = max(pmax[1], curp[1]);
        pmax[2] = max(pmax[2], curp[2]);
    }

    center = (pmin + pmax) * 0.5;
    halfSize = (pmax - pmin) * 0.5;
}
void PhysIKASystemBase::contactTest(const VPE::dvec3 &from, const VPE::dvec3 &to, const VPE::dvec3 &boxHalfExtents, RaytestResult &results, LayerMask mask = ALL_LAYERS) {
}
}  // namespace VPE