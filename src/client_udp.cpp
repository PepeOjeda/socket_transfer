#include "MinimalSocket/udp/UdpSocket.h"
#include "Serialization.hpp"
#include <rclcpp/rclcpp.hpp>

using sensor_msgs::msg::CompressedImage;

class ClientUDP : public rclcpp::Node
{
public:
    ClientUDP();

private:
    void ImageCallback(const CompressedImage::SharedPtr msg);

private:
    rclcpp::Subscription<CompressedImage>::SharedPtr subscription;
    MinimalSocket::udp::Udp<true> socket;
    std::optional<MinimalSocket::Address> serverAddress;

    std::vector<char> buffer;
    uint8_t imageID = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientUDP>();
    rclcpp::spin(node);
    return 0;
}

ClientUDP::ClientUDP()
    : Node("image_transport_client")
{
    std::string topic = declare_parameter("topic", "/rgbd/color/compressed");
    subscription = create_subscription<CompressedImage>(topic, 1, std::bind(&ClientUDP::ImageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on topic '%s'", topic.c_str());

    std::string serverIP = declare_parameter("serverIP", "127.0.0.1");
    MinimalSocket::Port serverPort = declare_parameter("serverPort", 15768);
    serverAddress.emplace(serverIP, serverPort);
    socket.open();
    RCLCPP_INFO(get_logger(), "Sending images to %s:%d", serverIP.c_str(), serverPort);

    buffer.resize(bufferSize);
}

void ClientUDP::ImageCallback(const CompressedImage::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "Got image!");
    MinimalSocket::BufferView bufferView;
    bufferView.buffer = buffer.data();
    bufferView.buffer_size = buffer.size();
    std::vector<MinimalSocket::BufferViewConst> packetBufViews = Serialize(*msg, imageID, bufferView);

    for (auto view : packetBufViews)
    {
        // RCLCPP_INFO(get_logger(), "Sending packet");
        bool success = socket.sendTo(view, serverAddress.value());
        if (!success)
            RCLCPP_ERROR(get_logger(), "Failed to send message to %s:%d", serverAddress->getHost().c_str(), serverAddress->getPort());
        rclcpp::sleep_for(std::chrono::microseconds(1000));
    }

    imageID++;
    // rclcpp::sleep_for(std::chrono::seconds(2));
}
