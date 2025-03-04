#pragma once
#include "ActionMsg.hpp"
#include "socket_transfer/base/client_tcp.hpp"
#include "socket_transfer/base/node_udp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

namespace SocketTransfer
{

    template <typename Action>
    class ClientAction
    {
        using GoalMsg = GoalMsg<Action>;
        using FeedbackMsg = FeedbackMsg<Action>;

    public:
        ClientAction();
        void Run();

    private:
        void ServiceCallback(const std::shared_ptr<rmw_request_id_t> header, const typename Action::Request::SharedPtr request);
        void OnResponse(MinimalSocket::BufferView responseView);

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const typename Action::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle);
        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle);

    private:
        std::unique_ptr<SocketManager> client;
        rclcpp::Node::SharedPtr node;
        typename rclcpp_action::Server<Action>::SharedPtr actionServer;

        std::map<rclcpp_action::GoalUUID, const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>> activeGoals;
    };

    template <typename Action>
    inline ClientAction<Action>::ClientAction()
    {
        node = std::make_shared<rclcpp::Node>("client");

        std::string topic = node->declare_parameter("actionServer", "/actionServer");

        using namespace std::placeholders;
        // server name ("topic") uses the namespace of the node
        actionServer = rclcpp_action::create_server<Action>(node, topic,
                                                            std::bind(&ClientAction::handle_goal, this, _1, _2),
                                                            std::bind(&ClientAction::handle_cancel, this, _1),
                                                            std::bind(&ClientAction::handle_accepted, this, _1));

        RCLCPP_INFO(node->get_logger(), "Advertising action server '%s'", topic.c_str());

        std::string protocol = node->declare_parameter<std::string>("protocol", "UDP");
        if (protocol == "UDP")
            client = std::make_unique<NodeUDP>(node);
        else if (protocol == "TCP")
            client = std::make_unique<ClientTCPBase>(node);

        client->OnMessageCompleted = std::bind(&ClientAction<Action>::OnResponse, this, std::placeholders::_1);
    }

    template <typename Action>
    void ClientAction<Action>::Run()
    {
        client->Run();
    }

    template <typename Action>
    void ClientAction<Action>::OnResponse(MinimalSocket::BufferView bufferview)
    {
        FeedbackMsg response;
        Serializer<FeedbackMsg>::Deserialize(response, bufferview);

        try
        {
            auto goalHandle = activeGoals.at(response.uuid);
            activeGoals.erase(response.uuid);
            auto result = std::make_shared<Action::Result>(response.result);

            if (response.feedback == FeedbackMsg::Feedback::Canceled)
                goalHandle->canceled(result);
            else if (response.feedback == FeedbackMsg::Feedback::Completed)
                goalHandle->succeed(result);
            else
                RCLCPP_ERROR(node->get_logger(), "Invalid response type from server!");
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Caught exception while processing server response: '%s'", e.what());
        }
    }

    template <typename Action>
    inline rclcpp_action::GoalResponse ClientAction<Action>::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const typename Action::Goal> goal)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename Action>
    inline rclcpp_action::CancelResponse ClientAction<Action>::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle)
    {
        activeGoals.erase(goal_handle->get_goal_id());
        GoalMsg reqWithID{GoalMsg::MsgType::CancelGoal, goal_handle->get_goal_id()};
        client->SendMsg(reqWithID);

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename Action>
    inline void ClientAction<Action>::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle)
    {
        GoalMsg reqWithID{GoalMsg::MsgType::SendGoal, goal_handle->get_goal_id(), *goal_handle->get_goal()};
        client->SendMsg(reqWithID);
        activeGoals.insert(reqWithID.uuid, goal_handle);
    }

} // namespace SocketTransfer