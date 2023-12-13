#include "PhysiKaVehicleManager.h"

VPE::PhysiKaVehicleManager::PhysiKaVehicleManager() {
}

VPE::PhysiKaVehicleManager::~PhysiKaVehicleManager() {
}

void VPE::PhysiKaVehicleManager::AddCar(VPE::EntityID car_entt_id, std::shared_ptr<PhysIKA::Node> car_node, WHEELCOUNT w_c) {
}

void VPE::PhysiKaVehicleManager::AddCar(VPE::Entity car_entt, std::shared_ptr<PhysIKA::Node> car_node, WHEELCOUNT w_c) {
}

void VPE::PhysiKaVehicleManager::RemoveCar(VPE::EntityID car_entt_id) {
}

void VPE::PhysiKaVehicleManager::RemoveCar(VPE::Entity car_entt) {
}

void VPE::PhysiKaVehicleManager::MoveCar(VPE::EntityID car_entt_id, PHYCARMOVING command) {
}

void VPE::PhysiKaVehicleManager::MoveCar(VPE::Entity car_entt, PHYCARMOVING command) {
}
