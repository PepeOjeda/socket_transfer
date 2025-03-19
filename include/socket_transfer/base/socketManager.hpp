#pragma once
#include "../internals/utils.hpp"
#include "../serialization.hpp"
#include <MinimalSocket/tcp/TcpServer.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace SocketTransfer
{
    // functions to handle process termination by sending a "bye" message to the other socket
    // this allows us to stop and restart nodes in one device without having to re-launch in the other device
    // we have to do this a little weird because an instance method cannot be hooked up to a POSIX signal directly
    namespace Internal
    {
        inline std::function<void(void)> onSigterm;
        inline void handleTermination()
        {
            std::fprintf(stderr, "Sending bye\n");

            onSigterm();
            rclcpp::shutdown();
        }

        inline void handleTermination(int signum)
        {
            handleTermination();
        }
    } // namespace Internal

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
        virtual size_t ReceivePeek(MinimalSocket::BufferView buffer) = 0;
        virtual bool Send(MinimalSocket::BufferView messageView) = 0;

        void SendControlMsg(Internal::PacketHeader::MsgType type);

    private:
        void ListenSocket(MinimalSocket::BufferView& currentInputBufferView);
        bool ByeReceived(MinimalSocket::BufferView currentInputBufferView);
        void SendBye();
        void Spin();

    protected:
        rclcpp::Node::SharedPtr node;

    private:
        std::vector<char> inputBuffer;
        MinimalSocket::BufferView currentInputBufferView;
        std::optional<Internal::Message> currentReceivedMessage;
        std::optional<rclcpp::executors::SingleThreadedExecutor> exec;

        uint8_t outputMessageID = 0;
        std::vector<char> outputBuffer;
        bool running = true;
    };

    /* Server implementation */

    inline SocketManager::SocketManager(rclcpp::Node::SharedPtr _node)
        : node(_node)
    {
        inputBuffer.resize(bufferSize);
        outputBuffer.resize(bufferSize);
        currentInputBufferView = {inputBuffer.data(), packetSize};

        // create executor
        rclcpp::ExecutorOptions options;
        options.context = node->get_node_base_interface()->get_context();
        exec.emplace(options);
        exec->add_node(node);

        // hook up the termination logic
        signal(SIGTERM, Internal::handleTermination);
        signal(SIGINT, Internal::handleTermination);
        signal(SIGHUP, Internal::handleTermination);
        atexit(Internal::handleTermination);
        Internal::onSigterm = std::bind(&SocketManager::SendBye, this);
    }

    inline void SocketManager::Run()
    {
        std::optional<std::thread> spinThread;

        // if the connection closes, just open the socket again
        while (rclcpp::ok() && running)
        {
            try
            {
                if (OpenSocket())
                    RCLCPP_INFO(node->get_logger(), "Ready for operation!");
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Could not open socket!");
                    exit(1);
                }

                if (!spinThread)
                    spinThread.emplace(std::bind(&SocketManager::Spin, this));

                ListenSocket(currentInputBufferView);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node->get_logger(), "Exception caught in run loop: %s", e.what());
            }

            exec->cancel();
            spinThread->join();
            spinThread.reset();
        }

        if (running)
            SendBye();

        RCLCPP_INFO(node->get_logger(), "Closing socketManager...");
        if (spinThread)
            spinThread->join();
    }

    template <typename Msg>
    void SocketManager::SendMsg(const Msg& msg)
    {
        static double maxFreq = Utils::getParam(node, "maxFrequency", 0);
        static rclcpp::Rate rate(maxFreq);
        try
        {
            MinimalSocket::BufferView serializedMsgBV;
            serializedMsgBV.buffer = outputBuffer.data();
            serializedMsgBV.buffer_size = outputBuffer.size();

            serializedMsgBV = Serializer<Msg>::Serialize(msg, serializedMsgBV);

            MinimalSocket::BufferView packetsBV;
            packetsBV.buffer = serializedMsgBV.buffer + serializedMsgBV.buffer_size;
            packetsBV.buffer_size = outputBuffer.size() - serializedMsgBV.buffer_size;

            std::vector<Internal::Packet> packets = DividePackets(serializedMsgBV, outputMessageID, packetsBV);

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
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Caught exception sending msg: '%s'", e.what());
        }

        if (maxFreq > 0)
            rate.sleep();
    }

    inline void SocketManager::ListenSocket(MinimalSocket::BufferView& currentInputBufferView)
    {
        while (rclcpp::ok() && running)
        {
            // read from the message itself the size of the packet, to avoid reading past the end of a small packet and into the beginning of the next one
            currentInputBufferView.buffer_size = sizeof(Internal::PacketHeader::packetSize);
            {
                size_t received_bytes = ReceivePeek(currentInputBufferView);
                currentInputBufferView.buffer_size = *(uint16_t*)currentInputBufferView.buffer;
            }

            if (!running)
                return;

            size_t packetSize = currentInputBufferView.buffer_size;
            size_t received_bytes = Receive(currentInputBufferView);
            while (received_bytes < packetSize && rclcpp::ok())
            {
                RCLCPP_WARN(node->get_logger(), "Expected %zu bytes, but only got %zu! Trying to read the remaining bytes...", currentInputBufferView.buffer_size, received_bytes);
                MinimalSocket::BufferView missingBytesView{currentInputBufferView.buffer + received_bytes, currentInputBufferView.buffer_size - received_bytes};
                received_bytes += Receive(missingBytesView);
            }

            Internal::Packet packet = ReadPacketAndAdvance(currentInputBufferView, packetSize);
            if (packet.header.msgType == Internal::PacketHeader::MsgType::Bye)
            {
                RCLCPP_WARN(node->get_logger(), "Received bye message");
                return;
            }
            else if (packet.header.msgType != Internal::PacketHeader::MsgType::Data)
            {
                RCLCPP_WARN_STREAM(node->get_logger(), "Ignoring message:" << packet.header);
                continue;
            }

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

    inline bool SocketManager::ByeReceived(MinimalSocket::BufferView bufferV)
    {
        if (bufferV.buffer_size == sizeof(Internal::PacketHeader))
        {
            Internal::PacketHeader header = Internal::ReadHeader(bufferV);

            if (header.msgType == Internal::PacketHeader::MsgType::Bye)
            {
                RCLCPP_WARN(node->get_logger(), "Received 'bye' msg, resetting socketManager.");
                return true;
            }
        }
        return false;
    }

    inline void SocketManager::SendBye()
    {
        try
        {
            running = false;
            SendControlMsg(Internal::PacketHeader::MsgType::Bye);
            std::fprintf(stderr, "Sent bye\n");
        }
        catch (std::exception& e)
        {
            std::fprintf(stderr, "Caught exception while trying to send 'bye': '%s'\n", e.what());
        }
    }

    inline void SocketManager::Spin()
    {
        exec->spin();
    }

    inline void SocketManager::SendControlMsg(Internal::PacketHeader::MsgType type)
    {
        Internal::PacketHeader msg{};
        msg.packetSize = sizeof(Internal::PacketHeader);
        msg.msgType = type;

        Send({.buffer = (char*)&msg, .buffer_size = sizeof(Internal::PacketHeader)});
    }
} // namespace SocketTransfer
