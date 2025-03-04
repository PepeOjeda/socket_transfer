#pragma once
#include "socket_transfer/base/node_udp.hpp"
#include "socket_transfer/base/server_tcp.hpp"

namespace SocketTransfer
{

    template <typename Msg>
    class ServerTopic
    {
    public:
        ServerTopic();
        void Run();

    private:
        std::unique_ptr<SocketManager> server;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rclcpp::Publisher<Msg>> pub;
    };

    template <typename Msg>
    inline ServerTopic<Msg>::ServerTopic()
    {
        node = std::make_shared<rclcpp::Node>("server");

        std::string topic = node->declare_parameter("topic", "/topic");
        pub = node->create_publisher<Msg>(topic, 1);
        RCLCPP_INFO(node->get_logger(), "Publishing to topic '%s'", pub->get_topic_name());

        std::string protocol = node->declare_parameter<std::string>("protocol", "UDP");
        if (protocol == "UDP")
            server = std::make_unique<NodeUDP>(node);
        else if (protocol == "TCP")
            server = std::make_unique<ServerTCPBase>(node);

        server->OnMessageCompleted = [&](MinimalSocket::BufferView bufView)
        {
            RCLCPP_INFO(node->get_logger(), "Received message!");
            Msg msg;
            Serializer<Msg>::Deserialize(msg, bufView);
            pub->publish(msg);
        };
    }

    template <typename Msg>
    inline void ServerTopic<Msg>::Run()
    {
        server->Run();
    }
} // namespace SocketTransfer