#pragma once
#include "socket_transfer/base/socketManager.hpp"
#include "socket_transfer/internals/utils.hpp"
#include <MinimalSocket/tcp/TcpServer.h>

namespace SocketTransfer
{
    class ServerTCPBase : public SocketManager
    {
        using SocketManager::SocketManager;

    protected:
        bool OpenSocket() override;
        size_t Receive(MinimalSocket::BufferView buffer) override;
        size_t ReceivePeek(MinimalSocket::BufferView buffer) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    private:
        void AcceptConnection();

    private:
        MinimalSocket::tcp::TcpServer<true> socket;
        std::optional<MinimalSocket::tcp::TcpConnectionBlocking> accepted_connection;
    };

    /* Server implementation */

    inline bool ServerTCPBase::OpenSocket()
    {
        MinimalSocket::Port port = this->node->declare_parameter<MinimalSocket::Port>("port", 15768);
        socket = {port, MinimalSocket::AddressFamily::IP_V4};

        if (socket.open())
        {
            RCLCPP_INFO(node->get_logger(), "Listening on port %d", port);
            socket.setBufferSize(10e6);
            AcceptConnection();
            return true;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Could not open socket on port %d", port);
            return false;
        }
    }

    inline size_t ServerTCPBase::Receive(MinimalSocket::BufferView buffer)
    {
        return accepted_connection->receive(buffer);
    }

    inline size_t ServerTCPBase::ReceivePeek(MinimalSocket::BufferView buffer)
    {
        auto received = accepted_connection->peek(buffer);
        return received;
    }

    inline bool ServerTCPBase::Send(MinimalSocket::BufferView messageView)
    {
        return accepted_connection->send(AsConst(messageView));
    }

    inline void ServerTCPBase::AcceptConnection()
    {
        if (!accepted_connection)
        {
            accepted_connection = socket.acceptNewClient();
            RCLCPP_INFO(node->get_logger(), "Accepted connection from %s:%d",
                        accepted_connection->getRemoteAddress().getHost().c_str(),
                        accepted_connection->getRemoteAddress().getPort());
        }
    }

} // namespace SocketTransfer
