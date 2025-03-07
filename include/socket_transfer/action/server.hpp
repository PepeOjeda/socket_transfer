#pragma once
#include "ActionMsg.hpp"
#include "socket_transfer/action/utils.hpp"
#include "socket_transfer/internals/create_socket.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

namespace SocketTransfer
{

    template <typename Action>
    class ServerAction
    {
        using GoalMsgT = GoalMsg<Action>;
        using FeedbackMsgT = FeedbackMsg<Action>;

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
        std::map<rclcpp_action::GoalUUID, const std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>>> originalUUIDToGoal;
        std::map<rclcpp_action::GoalUUID, rclcpp_action::GoalUUID> localUUIDToOriginal;
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
        FeedbackMsgT msg;
        msg.uuid = localUUIDToOriginal.at(w_result.goal_id);
        msg.result = *w_result.result;
        msg.feedback = w_result.code == rclcpp_action::ResultCode::SUCCEEDED ? FeedbackMsgT::Feedback::Completed : FeedbackMsgT::Feedback::Canceled;

        originalUUIDToGoal.erase(msg.uuid);
        localUUIDToOriginal.erase(w_result.goal_id);

        std::string uuidStr = Utils::UUIDasString(msg.uuid);
        RCLCPP_INFO(node->get_logger(), "Sending result for goal %s", uuidStr.c_str());
        socketManager->SendMsg(msg);
    }

    template <typename Action>
    inline void ServerAction<Action>::OnMessageComplete(MinimalSocket::BufferView bufView)
    {
        GoalMsgT request;
        Serializer<GoalMsgT>::Deserialize(request, bufView);

        std::string uuidStr = Utils::UUIDasString(request.uuid);
        RCLCPP_INFO(node->get_logger(), "Received request for goal %s", uuidStr.c_str());

        if (request.type == GoalMsgT::MsgType::SendGoal)
        {
            typename rclcpp_action::Client<Action>::SendGoalOptions goal_options;
            goal_options.result_callback = std::bind(&ServerAction<Action>::OnActionResult, this, std::placeholders::_1);
            goal_options.goal_response_callback = [&](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goalHandle)
            {
                originalUUIDToGoal.insert({request.uuid, goalHandle});
                localUUIDToOriginal.insert({goalHandle->get_goal_id(), request.uuid});
            };
            client->async_send_goal(request.goal, goal_options);
        }
        else if (request.type == GoalMsgT::MsgType::CancelGoal)
        {
            try
            {
                client->async_cancel_goal(originalUUIDToGoal.at(request.uuid));
                originalUUIDToGoal.erase(request.uuid);
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