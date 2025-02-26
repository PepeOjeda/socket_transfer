#pragma once
#include "MinimalSocket/udp/UdpSocket.h"
#include "serialization.hpp"
#include "utils.hpp"
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

template <typename Msg>
class ClientUDP : public rclcpp::Node
{
public:
    ClientUDP();

private:
    void ImageCallback(const typename Msg::SharedPtr msg);

private:
    typename rclcpp::Subscription<Msg>::SharedPtr subscription;
    MinimalSocket::udp::Udp<true> socket;
    std::optional<MinimalSocket::Address> serverAddress;

    std::vector<char> buffer;
    uint8_t imageID = 0;
};

template <typename Msg>
ClientUDP<Msg>::ClientUDP()
    : Node("image_transport_client")
{
    std::string topic = declare_parameter("topic", "/rgbd/color/compressed");
    subscription = create_subscription<Msg>(topic, 1, std::bind(&ClientUDP::ImageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on topic '%s'", topic.c_str());

    std::string serverIP = declare_parameter("serverIP", "127.0.0.1");
    MinimalSocket::Port serverPort = declare_parameter("serverPort", 15768);
    serverAddress.emplace(serverIP, serverPort);

    socket.setBufferSize(10e5);
    socket.open();
    RCLCPP_INFO(get_logger(), "Sending images to %s:%d", serverIP.c_str(), serverPort);

    buffer.resize(bufferSize);
}

template <typename Msg>
void ClientUDP<Msg>::ImageCallback(const typename Msg::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "Got image!");

    MinimalSocket::BufferView bufferView;
    bufferView.buffer = buffer.data();
    bufferView.buffer_size = buffer.size();
    std::vector<Packet> packets = Serialize(*msg, imageID, bufferView);

    for (auto packet : packets)
    {
        if (!rclcpp::ok())
            exit(-1);
        // RCLCPP_INFO(get_logger(), "Sending packet Image %d: %d/%d, %ld bytes",
        //             packet.header.imageID,
        //             packet.header.packetID,
        //             packet.header.numPackets - 1,
        //             packet.data.buffer_size);
        bool success = socket.sendTo(AsConst(packet.data), *serverAddress);
        if (!success)
            RCLCPP_ERROR(get_logger(), "Failed to send message to %s:%d", serverAddress->getHost().c_str(), serverAddress->getPort());
        // rclcpp::sleep_for(std::chrono::microseconds(200));
    }
    imageID++;
}
