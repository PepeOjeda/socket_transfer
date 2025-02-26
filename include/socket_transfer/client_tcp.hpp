#pragma once
#include "MinimalSocket/tcp/TcpClient.h"
#include "serialization.hpp"
#include "utils.hpp"
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

template <typename Msg>
class ClientTCP : public rclcpp::Node
{
public:
    ClientTCP();

private:
    void MsgCallback(const typename Msg::SharedPtr msg);

private:
    typename rclcpp::Subscription<Msg>::SharedPtr subscription;
    std::optional<MinimalSocket::tcp::TcpClient<true>> socket;
    std::optional<MinimalSocket::Address> serverAddress;

    std::vector<char> buffer;
    uint8_t messageID = 0;
};

template <typename Msg>
ClientTCP<Msg>::ClientTCP()
    : Node("socket_transport_client")
{
    std::string topic = declare_parameter("topic", "/rgbd/color/compressed");
    subscription = create_subscription<Msg>(topic, 1, std::bind(&ClientTCP::MsgCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on topic '%s'", topic.c_str());

    std::string serverIP = declare_parameter("serverIP", "127.0.0.1");
    MinimalSocket::Port serverPort = declare_parameter("serverPort", 15768);
    serverAddress.emplace(serverIP, serverPort);

    socket.emplace(*serverAddress);
    socket->setBufferSize(10e5);
    socket->open();
    RCLCPP_INFO(get_logger(), "Sending messages to %s:%d", serverIP.c_str(), serverPort);

    buffer.resize(bufferSize);
}

template <typename Msg>
void ClientTCP<Msg>::MsgCallback(const typename Msg::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "Got message!");

    MinimalSocket::BufferView serializedMsgBV;
    serializedMsgBV.buffer = buffer.data();
    serializedMsgBV.buffer_size = buffer.size();

    serializedMsgBV = Serialize(*msg, serializedMsgBV);

    MinimalSocket::BufferView packetsBV;
    packetsBV.buffer = serializedMsgBV.buffer + serializedMsgBV.buffer_size;
    packetsBV.buffer_size = buffer.size() - serializedMsgBV.buffer_size;

    std::vector<Packet> packets = DividePackets(serializedMsgBV, messageID, packetsBV);

    for (auto packet : packets)
    {
        if (!rclcpp::ok())
            exit(-1);
        // RCLCPP_INFO(get_logger(), "Sending packet message %d: %d/%d, %ld bytes",
        //             packet.header.messageID,
        //             packet.header.packetID,
        //             packet.header.numPackets - 1,
        //             packet.data.buffer_size);
        bool success = socket->send(AsConst(packet.data));
        if (!success)
            RCLCPP_ERROR(get_logger(), "Failed to send message to %s:%d", serverAddress->getHost().c_str(), serverAddress->getPort());
        // rclcpp::sleep_for(std::chrono::microseconds(200));
    }
    messageID++;
}
