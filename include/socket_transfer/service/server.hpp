#pragma once
#include "socket_transfer/internals/create_socket.hpp"
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
        std::unique_ptr<SocketManager> socketManager;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rclcpp::Client<Msg>> client;
    };

    template <typename Msg>
    inline ServerService<Msg>::ServerService()
    {
        node = std::make_shared<rclcpp::Node>("server");

        std::string topic = Utils::getParam<std::string>(node, "service", "/topic");
        client = node->create_client<Msg>(topic);
        RCLCPP_INFO(node->get_logger(), "Communicating with service '%s'", client->get_service_name());

        CreateSocket(node, socketManager);

        socketManager->OnMessageCompleted = [&](MinimalSocket::BufferView bufView)
        {
            Request request;
            Serializer<Request>::Deserialize(request, bufView);
            
            //we need the actual ros request to be a shared ptr to pass it into async_send_request
            auto ros_request = std::shared_ptr<typename Msg::Request>();
            *ros_request = request.request;

            auto future = client->async_send_request(ros_request);
            auto result = rclcpp::spin_until_future_complete(node, future);
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto ros_response = future.get();
                Response response{request.header, *ros_response};
                socketManager->SendMsg(response);
            }
        };
    }

    template <typename Msg>
    inline void ServerService<Msg>::Run()
    {
        socketManager->Run();
    }
} // namespace SocketTransfer