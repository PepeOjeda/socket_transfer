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
    struct Serializer<GoalMsg<ActionT>>
    {
        static MinimalSocket::BufferView Serialize(const GoalMsg<ActionT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.type);
            writer.Write(&msg.uuid);

            return Serializer<typename ActionT::Goal>::Serialize(msg.goal, writer.getRemainingBuffer());
        }

        static void Deserialize(GoalMsg<ActionT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.type);
            reader.Read(&msg.uuid);

            Serializer<typename ActionT::Goal>::Deserialize(msg.goal, reader.getRemainingBuffer());
        }
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

    template <typename ActionT>
    struct Serializer<FeedbackMsg<ActionT>>
    {
        static MinimalSocket::BufferView Serialize(const FeedbackMsg<ActionT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.feedback);
            writer.Write(&msg.uuid);

            return Serializer<typename ActionT::Goal>::Serialize(msg.goal, writer.getRemainingBuffer());
        }

        static void Deserialize(FeedbackMsg<ActionT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.feedback);
            reader.Read(&msg.uuid);

            Serializer<typename ActionT::Goal>::Deserialize(msg.goal, reader.getRemainingBuffer());
        }
    };
} // namespace SocketTransfer