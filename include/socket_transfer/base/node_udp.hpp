#pragma once
#include "socketManager.hpp"
#include "socket_transfer/utils.hpp"
#include <MinimalSocket/udp/UdpSocket.h>

namespace SocketTransfer
{
    class NodeUDP : public SocketManager
    {
    protected:
        virtual bool OpenSocket() override;
        virtual size_t Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    protected:
        std::optional<MinimalSocket::Address> otherNodeAddress;

    private:
        std::optional<MinimalSocket::udp::Udp<true>> socket;
    };

    /* Server implementation */

    inline bool NodeUDP::OpenSocket()
    {
        MinimalSocket::Port port = node->template declare_parameter<MinimalSocket::Port>("port", 15768);
        socket.emplace(port);
        socket->setBufferSize(10e5);

        if (socket->open())
        {
            RCLCPP_INFO(node->get_logger(), "Listening on port %d", port);
            return true;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Could not open socket on port %d", port);
            return false;
        }

        MinimalSocket::Port serverPort = node->template declare_parameter<MinimalSocket::Port>("serverPort", 15768);
        std::string serverIP = node->template declare_parameter<std::string>("serverIP", "127.0.0.1");
        otherNodeAddress = {serverIP, serverPort};
    }

    inline size_t NodeUDP::Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout)
    {
        auto received = socket->receive(buffer, timeout);
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
