#pragma once

#include "PhysiKaNetData.h"
#include "DataBuffer.h"

class PhyCommandDecoder {
public:
    PhyCommandDecoder(const VSVLink::DataBuffer &buf)
        : buffer_(buf.data, buf.size), read_pos(0) {
    }

    ~PhyCommandDecoder() {
    }

    std::pair<PHYSIKA_NET_DATA_TYPE, VSVLink::ConstBuffer> NextCommand() {
        auto result = std::make_pair(PHYSIKA_NET_DATA_TYPE::PHY_NET_DATA_TYPE_START, VSVLink::ConstBuffer());

        std::size_t const remaining_size = buffer_.size - read_pos;
        if (remaining_size < sizeof(PhysiKaCmdHeader))
            return result;

        PhysiKaCmdHeader const *header = reinterpret_cast<PhysiKaCmdHeader const *>(buffer_.data + read_pos);
        if (header->cmd_size > remaining_size)
            return result;

        result.first = PHYSIKA_NET_DATA_TYPE(header->cmd_type);
        result.second = VSVLink::ConstBuffer(buffer_.data + read_pos, std::size_t(header->cmd_size));

        read_pos += std::size_t(header->cmd_size);
        return result;
    }

    void Reset() {
        read_pos = 0;
    }

private:
    VSVLink::ConstBuffer buffer_;
    std::size_t read_pos = 0;
};