#pragma once
#include "socket_transfer/base/client_tcp.hpp"
#include "socket_transfer/base/node_udp.hpp"

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
        std::unique_ptr<SocketManager> client;
        rclcpp::Node::SharedPtr node;
        typename rclcpp::Subscription<Msg>::SharedPtr sub;
    };

    template <typename Msg>
    inline ClientTopic<Msg>::ClientTopic()
    {
        node = std::make_shared<rclcpp::Node>("client");
        
        std::string topic = node->declare_parameter("topic", "/topic");
        sub = node->create_subscription<Msg>(topic, 1, std::bind(&ClientTopic<Msg>::MsgCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(), "Listening to topic %s", sub->get_topic_name());

        std::string protocol = node->declare_parameter<std::string>("protocol", "UDP");
        if (protocol == "UDP")
            client = std::make_unique<NodeUDP>(node);
        else if (protocol == "TCP")
            client = std::make_unique<ClientTCPBase>(node);
    }

    template <typename Msg>
    inline void ClientTopic<Msg>::Run()
    {
        client->Run();
    }

    template <typename Msg>
    inline void ClientTopic<Msg>::MsgCallback(const typename Msg::SharedPtr msg)
    {
        client->SendMsg(*msg);
    }
} // namespace SocketTransfer