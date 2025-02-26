#pragma once
#include <MinimalSocket/tcp/TcpServer.h>
#include <rclcpp/rclcpp.hpp>
#include "serialization.hpp"


template <typename Msg>
class ServerTCP : public rclcpp::Node
{
public:
    ServerTCP();
    void Run();

private:
    MinimalSocket::Port port;
    MinimalSocket::tcp::TcpServer<true> socket;
    typename rclcpp::Publisher<Msg>::SharedPtr publisher;
    std::optional<Message> currentMessage;
};


/* Server implementation */

template <typename Msg>
ServerTCP<Msg>::ServerTCP()
    : Node("socket_transport_server")
{
    port = declare_parameter<MinimalSocket::Port>("port", 15768);
    socket = {port, MinimalSocket::AddressFamily::IP_V4};
    socket.setBufferSize(10e5);
    if (socket.open())
        RCLCPP_INFO(get_logger(), "Listening on port %d", port);
    else
    {
        RCLCPP_ERROR(get_logger(), "Could not open UDP socket on port %d", port);
        exit(-1);
    }

    std::string topic = declare_parameter("topic", "receivedMessage");
    publisher = create_publisher<Msg>(topic, 1);
    RCLCPP_INFO(get_logger(), "Publishing messages on topic '%s'", publisher->get_topic_name());
}

template <typename Msg>
void ServerTCP<Msg>::Run()
{
    MinimalSocket::tcp::TcpConnectionBlocking accepted_connection =
    socket.acceptNewClient();

    std::vector<char> buffer(bufferSize, 0);
    MinimalSocket::BufferView bufferView{.buffer = buffer.data(), .buffer_size = bufferSize};

    while (rclcpp::ok())
    {
        size_t receivedBytes = accepted_connection.receive(bufferView);
        if (!receivedBytes)
        {
            RCLCPP_ERROR(get_logger(), "Invalid result from socket.receive(). This should never happen with a blocking socket.");
            continue;
        }

        Packet packet = ReadPacketAndAdvance(bufferView, receivedBytes);
        // RCLCPP_INFO(get_logger(), "Received packet! message %d: %d/%d, %ld bytes",
        //             packet.header.messageID,
        //             packet.header.packetID,
        //             packet.header.numPackets - 1,
        //             receivedMessage->received_bytes);

        if (!currentMessage)
            currentMessage.emplace();
        else if (currentMessage->messageID() != packet.header.messageID)
        {
            RCLCPP_WARN(get_logger(), "Discarding message %d, we received a packet for message %d", currentMessage->messageID(), packet.header.messageID);
            bufferView = {.buffer = buffer.data(), .buffer_size = bufferSize};
            RelocatePacket(packet, bufferView); // this new packet should now be moved to the beginning of the buffer, to avoid writing over it
            currentMessage.emplace();
        }

        currentMessage->packets.push_back(packet);

        if (currentMessage->isComplete())
        {
            Msg msg;
            ExtractData(currentMessage->packets, bufferView);
            Deserialize<Msg>(msg, bufferView);
            bufferView = {.buffer = buffer.data(), .buffer_size = bufferSize};
            publisher->publish(msg);
            
            currentMessage = std::nullopt;
            RCLCPP_INFO(get_logger(), "Message published!");
        }
    }
}
