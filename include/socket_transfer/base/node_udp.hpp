#pragma once
#include "socketManager.hpp"
#include "socket_transfer/utils.hpp"
#include <MinimalSocket/udp/UdpSocket.h>

namespace SocketTransfer
{
    class NodeUDP : public SocketManager
    {
        using SocketManager::SocketManager;

    protected:
        virtual bool OpenSocket() override;
        virtual size_t Receive(MinimalSocket::BufferView buffer) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    protected:
        std::optional<MinimalSocket::Address> otherNodeAddress;

    private:
        std::optional<MinimalSocket::udp::Udp<true>> socket;
    };

    /* Server implementation */

    inline bool NodeUDP::OpenSocket()
    {
        MinimalSocket::Port port = node->declare_parameter<MinimalSocket::Port>("port", MinimalSocket::ANY_PORT);
        socket.emplace(port);
        
        MinimalSocket::Port serverPort = node->declare_parameter<MinimalSocket::Port>("serverPort", 15768);
        std::string serverIP = node->declare_parameter<std::string>("serverIP", "127.0.0.1");
        otherNodeAddress = {serverIP, serverPort};

        if (socket->open())
        {
            RCLCPP_INFO(node->get_logger(), "Listening on port %d", socket->getPortToBind());
            socket->setBufferSize(10e6);
            return true;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Could not open socket on port %d", port);
            return false;
        }
    }

    inline size_t NodeUDP::Receive(MinimalSocket::BufferView buffer)
    {
        auto received = socket->receive(buffer);
        if (received)
        {
            if (!Equal(*otherNodeAddress, received->sender))
            {
                otherNodeAddress = received->sender;
                RCLCPP_WARN(node->get_logger(), "Changing other node address to %s:%d", otherNodeAddress->getHost().c_str(), otherNodeAddress->getPort());
            }
            return received->received_bytes;
        }
        else
            return {};
    }

    inline bool NodeUDP::Send(MinimalSocket::BufferView messageView)
    {
        return socket->sendTo(AsConst(messageView), *otherNodeAddress);
    }

} // namespace SocketTransfer
