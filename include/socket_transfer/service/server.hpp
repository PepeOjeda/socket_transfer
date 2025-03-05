#pragma once
#include "socket_transfer/base/node_udp.hpp"
#include "socket_transfer/base/server_tcp.hpp"
#include "socket_transfer/service/MsgWithHeader.hpp"

namespace SocketTransfer
{

    template <typename Msg>
    class ServerService
    {
        using Request = RequestMsg<Msg>;
        using Response = ResponseMsg<Msg>;

    public:
        ServerService();
        void Run();

    private:
        std::unique_ptr<SocketManager> server;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rclcpp::Client<Msg>> client;
    };

    template <typename Msg>
    inline ServerService<Msg>::ServerService()
    {
        node = std::make_shared<rclcpp::Node>("server");

        std::string topic = node->declare_parameter("service", "/topic");
        client = node->create_client<Msg>(topic, 1);
        RCLCPP_INFO(node->get_logger(), "Communicating with service '%s'", client->get_service_name());

        std::string protocol = node->declare_parameter<std::string>("protocol", "UDP");
        if (protocol == "UDP")
            server = std::make_unique<NodeUDP>(node);
        else if (protocol == "TCP")
            server = std::make_unique<ServerTCPBase>(node);

        server->OnMessageCompleted = [&](MinimalSocket::BufferView bufView)
        {
            Request request;
            Serializer<Request>::Deserialize(request, bufView);
            auto future = client->async_send_request(request);
            auto result = rclcpp::spin_until_future_complete(node, future);
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto ros_response = result.get();
                Response response{request.header, ros_response};
                server->SendMsg(response);
            }
        };
    }

    template <typename Msg>
    inline void ServerService<Msg>::Run()
    {
        server->Run();
    }
} // namespace SocketTransfer