#pragma once
#include "../internals/utils.hpp"
#include "MinimalSocket/tcp/TcpClient.h"
#include "socket_transfer/base/socketManager.hpp"

namespace SocketTransfer
{
    class ClientTCPBase : public SocketManager
    {
        using SocketManager::SocketManager;

    public:
        void Run();

    protected:
        bool OpenSocket() override;
        size_t Receive(MinimalSocket::BufferView buffer) override;
        size_t ReceivePeek(MinimalSocket::BufferView buffer) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    private:
        std::optional<MinimalSocket::tcp::TcpClient<true>> socket;
    };

    inline bool ClientTCPBase::OpenSocket()
    {
        MinimalSocket::Port serverPort = Utils::getParam<MinimalSocket::Port>(node, "serverPort", 15768);
        std::string serverIP = Utils::getParam<std::string>(node, "serverIP", "127.0.0.1");

        RCLCPP_INFO(node->get_logger(), "Trying to connect to server at %s:%d", serverIP.c_str(), serverPort);
        MinimalSocket::Address serverAddress{serverIP, serverPort};
        socket.emplace(serverAddress);

        // tcp client socket can only be opened once the server is ready, so we might need to try a few times
        while (!socket->wasOpened() && rclcpp::ok())
        {
            try
            {
                socket->open();
            }
            catch (const std::exception& e)
            {
                RCLCPP_INFO(node->get_logger(), "Could not connect to server at %s:%d, retrying...", serverIP.c_str(), serverPort);
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }

        if (socket->wasOpened())
        {
            RCLCPP_INFO(node->get_logger(), "Connected to server at %s:%d", serverIP.c_str(), serverPort);
            socket->setBufferSize(10e6);
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

    inline size_t ClientTCPBase::ReceivePeek(MinimalSocket::BufferView buffer)
    {
        auto received = socket->peek(buffer);
        return received;
    }

    inline bool ClientTCPBase::Send(MinimalSocket::BufferView messageView)
    {
        return socket->send(Utils::AsConst(messageView));
    }
} // namespace SocketTransfer