#pragma once
#include "MsgWithHeader.hpp"
#include "socket_transfer/internals/create_socket.hpp"

namespace SocketTransfer
{

    template <typename Msg>
    class ClientServer
    {
        using Request = RequestMsg<Msg>;
        using Response = ResponseMsg<Msg>;

    public:
        ClientServer();
        void Run();

    private:
        void ServiceCallback(const std::shared_ptr<rmw_request_id_t> header, const typename Msg::Request::SharedPtr request);
        void OnResponse(MinimalSocket::BufferView responseView);

    private:
        std::unique_ptr<SocketManager> socketManager;
        rclcpp::Node::SharedPtr node;
        typename rclcpp::Service<Msg>::SharedPtr service;
    };

    template <typename Msg>
    inline ClientServer<Msg>::ClientServer()
    {
        node = std::make_shared<rclcpp::Node>("client");

        std::string topic = Utils::getParam<std::string>(node, "service", "/topic");
        service = node->create_service<Msg>(topic, std::bind(&ClientServer<Msg>::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node->get_logger(), "Advertising service '%s'", service->get_service_name());

        CreateSocket(node, socketManager);

        socketManager->OnMessageCompleted = std::bind(&ClientServer<Msg>::OnResponse, this, std::placeholders::_1);
    }

    template <typename Msg>
    void ClientServer<Msg>::Run()
    {
        socketManager->Run();
    }

    template <typename Msg>
    void ClientServer<Msg>::ServiceCallback(const std::shared_ptr<rmw_request_id_t> header, const typename Msg::Request::SharedPtr request)
    {
        Request reqWithID{*header, *request};
        socketManager->SendMsg(reqWithID);
    }

    template <typename Msg>
    void ClientServer<Msg>::OnResponse(MinimalSocket::BufferView bufferview)
    {
        Response response;
        Serializer<Response>::Deserialize(response, bufferview);
        service->send_response(response.header, response.response);
    }

} // namespace SocketTransfer