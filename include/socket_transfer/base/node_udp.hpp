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
        size_t ReceiveWithTimeout(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout);
        void FlushInputBuffer();

    private:
        std::optional<MinimalSocket::udp::Udp<true>> socket;
    };

    /* Server implementation */

    inline bool NodeUDP::OpenSocket()
    {
        MinimalSocket::Port port = Utils::getParam<MinimalSocket::Port>(node, "port", MinimalSocket::ANY_PORT);
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
        return ReceiveWithTimeout(buffer, MinimalSocket::NULL_TIMEOUT);
    }

    inline size_t NodeUDP::ReceiveWithTimeout(MinimalSocket::BufferView buffer, MinimalSocket::Timeout timeout)
    {
        auto received = socket->receive(buffer, timeout);
        if (received)
        {
            if (!Utils::Equal(*otherNodeAddress, received->sender))
            {
                otherNodeAddress = received->sender;
                RCLCPP_WARN(node->get_logger(), "Changing other node address to %s:%d", otherNodeAddress->getHost().c_str(), otherNodeAddress->getPort());
            }
            return received->received_bytes;
        }
        else
            return 0;
    }

    inline void NodeUDP::FlushInputBuffer()
    {
        char temp[1000];
        while (ReceiveWithTimeout(MinimalSocket::BufferView{temp, 1000}, std::chrono::milliseconds(1)) != 0)
            ;
    }

    inline size_t NodeUDP::ReceivePeek(MinimalSocket::BufferView buffer)
    {
        auto received = socket->peek(buffer);
        return received;
    }

    inline bool NodeUDP::Send(MinimalSocket::BufferView messageView)
    {
        return socket->sendTo(Utils::AsConst(messageView), *otherNodeAddress);
    }

    inline void NodeUDP::InitializeCommunication()
    {
        bool isServer = Utils::getParam<bool>(node, "isServerSocket", false);

        // if this node is acting as server, wait until we get a message from the client (so we can know its address)
        if (isServer)
        {
            std::array<char, 100> buffer;
            MinimalSocket::BufferView bufferView{buffer.data(), buffer.size()};

            bool ready = false;
            while (!ready)
            {
                size_t received_bytes = Receive(bufferView);
                Internal::PacketHeader response = Internal::ReadHeader(bufferView);
                if (response.msgType == Internal::PacketHeader::MsgType::Hi)
                    ready = true;
                else
                {
                    RCLCPP_ERROR_STREAM(node->get_logger(), "Got weird first message from server! \nExpected:\t 'HI'\nGot:" << response);
                    continue;
                }
            }

            FlushInputBuffer();
            SendControlMsg(Internal::PacketHeader::MsgType::HiOK);
        }

        // otherwise, send a "hi" message to the server address and wait for an ack
        else
        {
            MinimalSocket::Port serverPort = Utils::getParam<MinimalSocket::Port>(node, "serverPort", 15768);
            std::string serverIP = Utils::getParam<std::string>(node, "serverIP", "127.0.0.1");
            otherNodeAddress = {serverIP, serverPort};

            RCLCPP_INFO(node->get_logger(), "Sending 'hi' message to server at %s:%d", serverIP.c_str(), serverPort);
            RCLCPP_INFO(node->get_logger(), "Waiting for server 'hiok'");

            std::array<char, 100> responseBuffer;
            MinimalSocket::BufferView bufferView{responseBuffer.data(), responseBuffer.size()};

            size_t receivedBytes = 0;
            do
            {
                SendControlMsg(Internal::PacketHeader::MsgType::Hi);
                receivedBytes = ReceiveWithTimeout(bufferView, std::chrono::milliseconds(500));
            } while (rclcpp::ok() && receivedBytes == 0);

            Internal::PacketHeader response = Internal::ReadHeader(bufferView);
            if (response.msgType != Internal::PacketHeader::MsgType::HiOK)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Got weird first message from server! \nExpected:\t 'HIOK'\nGot:" << response);
                exit(1);
            }
        }
    }

} // namespace SocketTransfer
