#pragma once
#include "socket_transfer/internals/create_socket.hpp"

namespace SocketTransfer
{

    template <typename Msg>
    class ClientTopic
    {
    public:
        ClientTopic();
        void Run();

    private:
        void MsgCallback(const typename Msg::SharedPtr msg);

    private:
        std::unique_ptr<SocketManager> socketManager;
        rclcpp::Node::SharedPtr node;
        typename rclcpp::Subscription<Msg>::SharedPtr sub;
    };

    template <typename Msg>
    inline ClientTopic<Msg>::ClientTopic()
    {
        node = std::make_shared<rclcpp::Node>("client");

        std::string topic = Utils::getParam<std::string>(node, "topic", "/topic");
        sub = node->create_subscription<Msg>(topic, 1, std::bind(&ClientTopic<Msg>::MsgCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(), "Listening to topic '%s'", sub->get_topic_name());

        CreateSocket(node, socketManager);
    }

    template <typename Msg>
    inline void ClientTopic<Msg>::Run()
    {
        socketManager->Run();
    }

    template <typename Msg>
    inline void ClientTopic<Msg>::MsgCallback(const typename Msg::SharedPtr msg)
    {
        RCLCPP_INFO(node->get_logger(), "Sending message");
        socketManager->SendMsg(*msg);
    }
} // namespace SocketTransfer