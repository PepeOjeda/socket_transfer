#pragma once
#include "ActionMsg.hpp"
#include "socket_transfer/internals/create_socket.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

namespace SocketTransfer
{

    template <typename Action>
    class ServerAction
    {
        using GoalMsg = GoalMsg<Action>;
        using FeedbackMsg = FeedbackMsg<Action>;

    public:
        ServerAction();
        void Run();

    private:
        void OnActionResult(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult& w_result);
        void OnMessageComplete(MinimalSocket::BufferView bufView);

    private:
        std::unique_ptr<SocketManager> socketManager;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rclcpp_action::Client<Action>> client;
        std::map<rclcpp_action::GoalUUID, const std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>>> activeGoals;
    };

    template <typename Action>
    inline ServerAction<Action>::ServerAction()
    {
        node = std::make_shared<rclcpp::Node>("server");

        std::string topic = node->declare_parameter("actionServer", "/topic");
        client = rclcpp_action::create_client<Action>(node, topic);
        RCLCPP_INFO(node->get_logger(), "Communicating with actionServer '%s'", topic.c_str());
        CreateSocket(node, socketManager);

        socketManager->OnMessageCompleted = std::bind(&ServerAction<Action>::OnMessageComplete, this, std::placeholders::_1);
    }

    template <typename Action>
    inline void ServerAction<Action>::Run()
    {
        socketManager->Run();
    }

    template <typename Action>
    inline void ServerAction<Action>::OnActionResult(const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult& w_result)
    {
        FeedbackMsg msg;
        msg.uuid = w_result.goal_id;
        msg.result = *w_result.result;
        msg.feedback = w_result.code == rclcpp_action::ResultCode::SUCCEEDED ? FeedbackMsg::Feedback::Completed : FeedbackMsg::Feedback::Canceled;

        activeGoals.erase(msg.uuid);

        socketManager->SendMsg(msg);
    }

    template <typename Action>
    inline void ServerAction<Action>::OnMessageComplete(MinimalSocket::BufferView bufView)
    {
        GoalMsg request;
        Serializer<GoalMsg>::Deserialize(request, bufView);

        if (request.type == GoalMsg::MsgType::SendGoal)
        {
            typename rclcpp_action::Client<Action>::SendGoalOptions goal_options;
            goal_options.result_callback = std::bind(&ServerAction<Action>::OnActionResult, this, std::placeholders::_1);
            goal_options.goal_response_callback = [&](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle)
            {
                activeGoals.insert({request.uuid, goalHandle});
            };
            client->async_send_goal(request.goal, goal_options);
        }
        else if (request.type == GoalMsg::MsgType::CancelGoal)
        {
            try
            {
                client->async_cancel_goal(activeGoals.at(request.uuid));
                activeGoals.erase(request.uuid);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node->get_logger(), "Caught exception: '%s", e.what());
            }
        }
        else
            RCLCPP_ERROR(node->get_logger(), "Invalid message type received from client");
    }
} // namespace SocketTransfer