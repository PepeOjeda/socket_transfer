#pragma once
#include "socket_transfer/internals/create_socket.hpp"

namespace SocketTransfer
{

    template <typename Msg>
    class ServerTopic
    {
    public:
        ServerTopic(const rclcpp::QoS& publisherQoS = 1);
        void Run();

    private:
        std::unique_ptr<SocketManager> socketManager;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rclcpp::Publisher<Msg>> pub;
    };

    template <typename Msg>
    inline ServerTopic<Msg>::ServerTopic(const rclcpp::QoS& publisherQoS)
    {
        node = std::make_shared<rclcpp::Node>("server");

        std::string topic = Utils::getParam<std::string>(node, "topic", "/topic");
        pub = node->create_publisher<Msg>(topic, publisherQoS);
        RCLCPP_INFO(node->get_logger(), "Publishing to topic '%s'", pub->get_topic_name());

        CreateSocket(node, socketManager);

        socketManager->OnMessageCompleted = [&](MinimalSocket::BufferView bufView)
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
        socketManager->Run();
    }
} // namespace SocketTransfer