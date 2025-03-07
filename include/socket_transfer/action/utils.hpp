#pragma once

#include <rclcpp_action/types.hpp>
#include <string>

namespace SocketTransfer::Utils
{
    inline std::string UUIDasString(const rclcpp_action::GoalUUID& uuid)
    {
        std::string str;
        str.resize(3*uuid.size());
        for(size_t i =0; i<uuid.size(); i++)
        {
            uint8_t elem = uuid.at(i);
            sprintf(str.data(), "%03d", elem);
        }
        return str;
    }
}