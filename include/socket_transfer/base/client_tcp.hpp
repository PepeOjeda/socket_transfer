#pragma once
#include "../utils.hpp"
#include "MinimalSocket/tcp/TcpClient.h"
#include "socket_transfer/base/socketManager.hpp"

namespace SocketTransfer
{
    class ClientTCPBase : public SocketManager
    {
    public:
        ClientTCPBase();
        void Run();

    protected:
        bool OpenSocket() override;
        size_t Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    private:
        std::optional<MinimalSocket::tcp::TcpClient<true>> socket;
    };

    inline bool ClientTCPBase::OpenSocket()
    {
        MinimalSocket::Port serverPort = this->node->template declare_parameter<MinimalSocket::Port>("serverPort", 15768);
        std::string serverIP = this->node->template declare_parameter<std::string>("serverIP", "127.0.0.1");

        MinimalSocket::Address serverAddress{serverIP, serverPort};
        socket.emplace(serverAddress);
        socket->setBufferSize(10e5);
        return socket->open();
    }

    inline size_t ClientTCPBase::Receive(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout)
    {
        return socket->receive(buffer, timeout);
    }

    inline bool ClientTCPBase::Send(MinimalSocket::BufferView messageView)
    {
        return socket->send(AsConst(messageView));
    }
} // namespace SocketTransfer