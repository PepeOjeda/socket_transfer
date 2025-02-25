#include "Serialization.hpp"
#include <MinimalSocket/udp/UdpSocket.h>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

using sensor_msgs::msg::CompressedImage;

// Gets compressed images from UDP socket, deserializes them, and publishes them on a ros2 topic
class ServerUDP : public rclcpp::Node
{
public:
    ServerUDP();
    void Run();

private:
    MinimalSocket::Port port;
    MinimalSocket::udp::Udp<true> socket;
    rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
    std::optional<Message> currentMessage;
};

/* Main */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServerUDP>();
    node->Run();
}

/* Server implementation */

ServerUDP::ServerUDP()
    : Node("image_transport_server")
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

    std::string topic = declare_parameter("topic", "receivedImage");
    publisher = create_publisher<CompressedImage>(topic, 1);
    RCLCPP_INFO(get_logger(), "Publishing images on topic '%s'", publisher->get_topic_name());
}

void ServerUDP::Run()
{
    std::vector<char> buffer(bufferSize, 0);
    MinimalSocket::BufferView bufferView{.buffer = buffer.data(), .buffer_size = bufferSize};

    while (rclcpp::ok())
    {
        auto start_t = std::clock();
        auto receivedMessage = socket.receive(bufferView);
        if (!receivedMessage)
        {
            RCLCPP_ERROR(get_logger(), "Invalid result from socket.receive(). This should never happen with a blocking socket.");
            continue;
        }

        Packet packet = ReadPacketAndAdvance(bufferView, receivedMessage->received_bytes);
        // RCLCPP_INFO(get_logger(), "Received packet! Image %d: %d/%d, %ld bytes",
        //             packet.header.imageID,
        //             packet.header.packetID,
        //             packet.header.numPackets - 1,
        //             receivedMessage->received_bytes);

        if (!currentMessage)
            currentMessage.emplace();
        else if (currentMessage->imageID() != packet.header.imageID)
        {
            RCLCPP_WARN(get_logger(), "Discarding message %d, we received a packet for message %d", currentMessage->imageID(), packet.header.imageID);
            bufferView = {.buffer = buffer.data(), .buffer_size = bufferSize};
            RelocatePacket(packet, bufferView); // this new packet should now be moved to the beginning of the buffer, to avoid writing over it
            currentMessage.emplace();
        }

        currentMessage->packets.push_back(packet);

        RCLCPP_INFO(get_logger(), "Ellapsed %fs", ((double)(std::clock() - start_t) / CLOCKS_PER_SEC));

        if (currentMessage->isComplete())
        {
            CompressedImage msg;
            Deserialize(msg, currentMessage->packets);
            publisher->publish(msg);

            currentMessage = std::nullopt;
            bufferView = {.buffer = buffer.data(), .buffer_size = bufferSize};
            RCLCPP_INFO(get_logger(), "Message published!");
        }
    }
}
