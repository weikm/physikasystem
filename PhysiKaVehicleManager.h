#pragma once

// STL
#include <unordered_map>

// VPE
#include "Entity.h"

// PhysIKA
#include "Physika/Framework/Framework/Node.h"

namespace VPE {

class PhysiKaVehicleManager {
public:
    enum class PHYCARMOVING {
        FORWARD,
        GOLEFT,
        GORIGHT,
        BACKWARD,
        INVALID_CONTROL
    };

    enum class WHEELCOUNT {
        PBD = 0,
        FOUR = 4,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        TEN,
        ETC
    };

public:
    PhysiKaVehicleManager();
    ~PhysiKaVehicleManager();

    void AddCar(VPE::EntityID car_entt_id, std::shared_ptr<PhysIKA::Node> car_node, WHEELCOUNT w_c = WHEELCOUNT::PBD);
    void AddCar(VPE::Entity car_entt, std::shared_ptr<PhysIKA::Node> car_node, WHEELCOUNT w_c = WHEELCOUNT::PBD);

    void RemoveCar(VPE::EntityID car_entt_id);
    void RemoveCar(VPE::Entity car_entt);

    void MoveCar(VPE::EntityID car_entt_id, PHYCARMOVING command);
    void MoveCar(VPE::Entity car_entt, PHYCARMOVING command);

private:
    std::unordered_map<VPE::EntityID, std::shared_ptr<PhysIKA::Node>> m_vehicle;

    std::unordered_map<VPE::EntityID, WHEELCOUNT> m_vehicleType;
};
}  // namespace VPE