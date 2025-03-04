#pragma once
#include "../utils.hpp"
#include "MinimalSocket/tcp/TcpClient.h"
#include "socket_transfer/base/socketManager.hpp"

namespace SocketTransfer
{
    class ClientTCPBase : public SocketManager
    {
        using SocketManager::SocketManager;

    public:
        ClientTCPBase();
        void Run();

    protected:
        bool OpenSocket() override;
        size_t Receive(MinimalSocket::BufferView buffer) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    private:
        std::optional<MinimalSocket::tcp::TcpClient<true>> socket;
    };

    inline bool ClientTCPBase::OpenSocket()
    {
        MinimalSocket::Port serverPort = this->node->declare_parameter<MinimalSocket::Port>("serverPort", 15768);
        std::string serverIP = this->node->declare_parameter<std::string>("serverIP", "127.0.0.1");

        RCLCPP_INFO(node->get_logger(), "Trying to connect to server at %s:%d", serverIP.c_str(), serverPort);
        MinimalSocket::Address serverAddress{serverIP, serverPort};
        socket.emplace(serverAddress);

        if (socket->open())
        {
            RCLCPP_INFO(node->get_logger(), "Connected to server at %s:%d", serverIP.c_str(), serverPort);
            socket->setBufferSize(10e5);
            return true;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Could not connect to server at %s:%d", serverIP.c_str(), serverPort);
            return false;
        }
    }

    inline size_t ClientTCPBase::Receive(MinimalSocket::BufferView buffer)
    {
        return socket->receive(buffer);
    }

    inline bool ClientTCPBase::Send(MinimalSocket::BufferView messageView)
    {
        return socket->send(AsConst(messageView));
    }
} // namespace SocketTransfer