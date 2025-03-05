#pragma once

#include "socket_transfer/base/client_tcp.hpp"
#include "socket_transfer/base/node_udp.hpp"
#include "socket_transfer/base/server_tcp.hpp"
#include "socket_transfer/base/socketManager.hpp"
#include <rclcpp/node.hpp>

namespace SocketTransfer
{
    inline void CreateSocket(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SocketManager>& manager)
    {
        std::string protocol = node->declare_parameter<std::string>("protocol", "UDP");
        if (protocol == "UDP")
            manager = std::make_unique<NodeUDP>(node);
        else if (protocol == "TCP")
        {
            bool isServer = node->declare_parameter<bool>("isServerSocket", false);
            if (isServer)
                manager = std::make_unique<ServerTCPBase>(node);
            else
                manager = std::make_unique<ClientTCPBase>(node);
        }
    }
} // namespace SocketTransfer