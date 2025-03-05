#pragma once
#include "socketManager.hpp"
#include "socket_transfer/internals/utils.hpp"
#include <MinimalSocket/udp/UdpSocket.h>

namespace SocketTransfer
{
    class NodeUDP : public SocketManager
    {
        using SocketManager::SocketManager;

    protected:
        virtual bool OpenSocket() override;
        virtual size_t Receive(MinimalSocket::BufferView buffer) override;
        virtual size_t ReceivePeek(MinimalSocket::BufferView buffer) override;
        bool Send(MinimalSocket::BufferView messageView) override;

    protected:
        std::optional<MinimalSocket::Address> otherNodeAddress;

    private:
        void InitializeCommunication();

    private:
        std::optional<MinimalSocket::udp::Udp<true>> socket;
    };

    /* Server implementation */

    inline bool NodeUDP::OpenSocket()
    {
        MinimalSocket::Port port = node->declare_parameter<MinimalSocket::Port>("port", MinimalSocket::ANY_PORT);
        socket.emplace(port);

        if (socket->open())
        {
            RCLCPP_INFO(node->get_logger(), "Listening on port %d", socket->getPortToBind());
            socket->setBufferSize(10e6);
            InitializeCommunication();
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
            return 0;
    }

    inline size_t NodeUDP::ReceivePeek(MinimalSocket::BufferView buffer)
    {
        auto received = socket->peek(buffer);
        return received;
    }

    inline bool NodeUDP::Send(MinimalSocket::BufferView messageView)
    {
        return socket->sendTo(AsConst(messageView), *otherNodeAddress);
    }

    inline void NodeUDP::InitializeCommunication()
    {
        bool isServer = node->declare_parameter<bool>("isServerSocket", false);

        // if this node is acting as server, wait until we get a message from the client (so we can know its address)
        if (isServer)
        {
            std::string buffer;
            buffer.resize(2);
            Receive(MinimalSocket::BufferView{buffer.data(), buffer.length()});
            if (buffer != "hi")
            {
                RCLCPP_ERROR(node->get_logger(), "Got weird first message from client! \nExpected:\t 'hi'\nGot\t '%s'", buffer.c_str());
                exit(1);
            }
        }

        // otherwise, send a "hi" message to the server address and wait for an ack
        else
        {
            MinimalSocket::Port serverPort = node->declare_parameter<MinimalSocket::Port>("serverPort", 15768);
            std::string serverIP = node->declare_parameter<std::string>("serverIP", "127.0.0.1");
            otherNodeAddress = {serverIP, serverPort};

            RCLCPP_INFO(node->get_logger(), "Sending hi message to server at %s:%d", serverIP.c_str(), serverPort);

            std::string hi("hi");
            Send(MinimalSocket::BufferView{hi.data(), hi.length()});
            
            std::string responseBuffer;
            responseBuffer.resize(4);
            Receive(MinimalSocket::BufferView{responseBuffer.data(), responseBuffer.length()});
            if(responseBuffer != "hiok")
            {
                RCLCPP_ERROR(node->get_logger(), "Got weird first message from server! \nExpected:\t 'hiok'\nGot\t '%s'", responseBuffer.c_str());
                exit(1);
            }
        }
    }

} // namespace SocketTransfer
