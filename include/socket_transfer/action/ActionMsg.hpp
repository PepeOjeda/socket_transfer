#pragma once

#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <socket_transfer/serialization.hpp>

namespace SocketTransfer
{
    template <typename ActionT>
    struct GoalMsg
    {
        enum MsgType : uint8_t
        {
            None = 0,
            SendGoal,
            CancelGoal
        };

        MsgType type;
        rclcpp_action::GoalUUID uuid;
        typename ActionT::Goal goal;
    };

    template <typename ActionT>
    struct FeedbackMsg
    {
        enum Feedback : uint8_t
        {
            None = 0,
            Completed,
            Canceled
        };

        Feedback feedback;
        rclcpp_action::GoalUUID uuid;
        typename ActionT::Result result;
    };

    template <typename Result>
    struct Serializer<GoalMsg<Result>>
    {
        static MinimalSocket::BufferView Serialize(const GoalMsg<Result>& msg, MinimalSocket::BufferView bufferView)
        {
            // serialize the common stuff ...

            return Serializer<Result>::Serialize(msg.result, bufferView);
        }
    };

    template <typename Result>
    struct Serializer<FeedbackMsg<Result>>
    {
        static MinimalSocket::BufferView Serialize(const FeedbackMsg<Result>& msg, MinimalSocket::BufferView bufferView)
        {
            // serialize the common stuff ...

            return Serializer<Result>::Serialize(msg.result, bufferView);
        }
    };
} // namespace SocketTransfer