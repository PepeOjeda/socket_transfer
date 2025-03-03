#pragma once
#include "socket_transfer/base/socketManager.hpp"
#include "socket_transfer/utils.hpp"
#include <MinimalSocket/tcp/TcpServer.h>

namespace SocketTransfer
{
    class ServerTCPBase : public SocketManager
    {
    protected:
        bool OpenSocket() override;
        size_t Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    private:
        MinimalSocket::tcp::TcpServer<true> socket;
        std::optional<MinimalSocket::tcp::TcpConnectionBlocking> accepted_connection;
    };

    /* Server implementation */

    inline bool ServerTCPBase::OpenSocket()
    {
        MinimalSocket::Port port = this->node->template declare_parameter<MinimalSocket::Port>("port", 15768);
        socket = {port, MinimalSocket::AddressFamily::IP_V4};
        socket.setBufferSize(10e5);
        return socket.open();
    }

    inline size_t ServerTCPBase::Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout)
    {
        if(!accepted_connection)
            accepted_connection = socket.acceptNewClient();
        return accepted_connection->receive(buffer, timeout);
    }

    inline bool ServerTCPBase::Send(MinimalSocket::BufferView messageView)
    {
        if(!accepted_connection)
            accepted_connection = socket.acceptNewClient();
        return accepted_connection->send(AsConst(messageView));
    }

} // namespace SocketTransfer
