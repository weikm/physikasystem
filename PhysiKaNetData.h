#pragma once

#include <stdio.h>

#include "VPMath.h"

#pragma pack(push)
#pragma pack(1)

enum PHYSIKA_NET_DATA_TYPE {
    PHY_NET_DATA_TYPE_START = 0,
    KEY_CONTROL,
    ENTITY_POS,
    ENTITY_SCALE,
    ENTITY_SENSOR,
    SENSOR_DATA,
    PHY_NET_DATA_TYPE_END
};

struct PhysiKaCmdHeader {
    PhysiKaCmdHeader(uint8_t type, uint16_t size) {
        cmd_type = type;
        cmd_size = size;
    }
    uint8_t cmd_type;   //命令类型CmdType枚举
    uint16_t cmd_size;  //整个包的大小
};

struct PhysiKaKeyboardCmd {
    PhysiKaCmdHeader header{PHYSIKA_NET_DATA_TYPE::KEY_CONTROL, sizeof(PhysiKaKeyboardCmd)};
    uint16_t entity_id;
    uint8_t key_val;
};

struct PhysiKaEntityPosCmd {
    PhysiKaCmdHeader header{PHYSIKA_NET_DATA_TYPE::ENTITY_POS, sizeof(PhysiKaEntityPosCmd)};
    uint16_t entity_id;
    double posx, posy, posz;
    double quax, quay, quaz, quaw;
};

struct PhysiKaEntityScaleCmd {
    PhysiKaCmdHeader header{PHYSIKA_NET_DATA_TYPE::ENTITY_SCALE, sizeof(PhysiKaEntityScaleCmd)};
    uint16_t entity_id;
    VPE::vec3 scale;
};

struct PhysiKaEntitySensorCmd {
    PhysiKaCmdHeader header{PHYSIKA_NET_DATA_TYPE::ENTITY_SENSOR, sizeof(PhysiKaEntitySensorCmd)};
    uint16_t entity_id;
    bool sensor_switch;
};

struct PhysiKaEntitySensorViewCmd {
    PhysiKaCmdHeader header{PHYSIKA_NET_DATA_TYPE::SENSOR_DATA, sizeof(PhysiKaEntitySensorViewCmd)};
    uint16_t entity_id;
};

#pragma pack(pop)