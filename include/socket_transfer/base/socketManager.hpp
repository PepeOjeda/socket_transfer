#pragma once
#include "../serialization.hpp"
#include <MinimalSocket/tcp/TcpServer.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace SocketTransfer
{
    class SocketManager
    {
    public:
        SocketManager(rclcpp::Node::SharedPtr _node);
        void Run();

        template <typename Msg>
        void SendMsg(const Msg& msg);
        std::function<void(MinimalSocket::BufferView)> OnMessageCompleted;

    protected:
        virtual bool OpenSocket() = 0;
        virtual size_t Receive(MinimalSocket::BufferView buffer) = 0;
        virtual bool Send(MinimalSocket::BufferView messageView) = 0;

    private:
        void ListenSocket(MinimalSocket::BufferView& currentInputBufferView);

    protected:
        rclcpp::Node::SharedPtr node;

    private:
        std::vector<char> inputBuffer;
        MinimalSocket::BufferView currentInputBufferView;
        std::optional<Message> currentReceivedMessage;

        uint8_t outputMessageID = 0;
        std::vector<char> outputBuffer;
    };

    /* Server implementation */

    inline SocketManager::SocketManager(rclcpp::Node::SharedPtr _node)
        : node(_node)
    {
        inputBuffer.resize(bufferSize);
        outputBuffer.resize(bufferSize);
        currentInputBufferView = {inputBuffer.data(), inputBuffer.size()};
    }

    inline void SocketManager::Run()
    {
        if (OpenSocket())
            RCLCPP_INFO(node->get_logger(), "Ready!");
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Could not open socket!");
            exit(1);
        }

        std::thread spinThread([&]()
                               {
                                   rclcpp::spin(node);
                               });
        ListenSocket(currentInputBufferView);

        spinThread.join();
    }

    template <typename Msg>
    void SocketManager::SendMsg(const Msg& msg)
    {
        MinimalSocket::BufferView serializedMsgBV;
        serializedMsgBV.buffer = outputBuffer.data();
        serializedMsgBV.buffer_size = outputBuffer.size();

        serializedMsgBV = Serialize(msg, serializedMsgBV);

        MinimalSocket::BufferView packetsBV;
        packetsBV.buffer = serializedMsgBV.buffer + serializedMsgBV.buffer_size;
        packetsBV.buffer_size = outputBuffer.size() - serializedMsgBV.buffer_size;

        std::vector<Packet> packets = DividePackets(serializedMsgBV, outputMessageID, packetsBV);

        for (auto packet : packets)
        {
            if (!rclcpp::ok())
                exit(-1);
            // RCLCPP_INFO(node->get_logger(), "Sending packet message %d: %d/%d, %ld bytes",
            //             packet.header.messageID,
            //             packet.header.packetID,
            //             packet.header.numPackets - 1,
            //             packet.data.buffer_size);
            bool success = Send(packet.data);
            if (!success)
                RCLCPP_ERROR(node->get_logger(), "Failed to send message!");
        }
        outputMessageID++;
    }

    inline void SocketManager::ListenSocket(MinimalSocket::BufferView& currentInputBufferView)
    {
        while (rclcpp::ok())
        {
            size_t received_bytes = Receive(currentInputBufferView);
            if (!received_bytes)
                return;

            Packet packet = ReadPacketAndAdvance(currentInputBufferView, received_bytes);
            // RCLCPP_INFO(node->get_logger(), "Received packet! message %d: %d/%d, %ld bytes",
            //             packet.header.messageID,
            //             packet.header.packetID,
            //             packet.header.numPackets - 1,
            //             received_bytes);

            if (!currentReceivedMessage)
                currentReceivedMessage.emplace();
            else if (currentReceivedMessage->messageID() != packet.header.messageID)
            {
                RCLCPP_WARN(node->get_logger(), "Discarding message %d, we received a packet for message %d", currentReceivedMessage->messageID(), packet.header.messageID);
                currentInputBufferView = {.buffer = inputBuffer.data(), .buffer_size = bufferSize};
                RelocatePacket(packet, currentInputBufferView); // this new packet should now be moved to the beginning of the buffer, to avoid writing over it
                currentReceivedMessage.emplace();
            }

            currentReceivedMessage->packets.push_back(packet);

            if (currentReceivedMessage->isComplete())
            {
                MinimalSocket::BufferView rawMsgView = ExtractData(currentReceivedMessage->packets, currentInputBufferView);
                OnMessageCompleted(rawMsgView);
                currentInputBufferView = {.buffer = inputBuffer.data(), .buffer_size = bufferSize};

                currentReceivedMessage = std::nullopt;
            }
        }
    }
} // namespace SocketTransfer
