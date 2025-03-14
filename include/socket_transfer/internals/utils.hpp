#pragma once
#include "MinimalSocket/core/Address.h"
#include "MinimalSocket/core/Definitions.h"
#include "rclcpp/node.hpp"

namespace SocketTransfer::Utils
{
    inline MinimalSocket::BufferViewConst AsConst(MinimalSocket::BufferView view)
    {
        return {.buffer = view.buffer, .buffer_size = view.buffer_size};
    }

    inline bool Equal(const MinimalSocket::Address& addr1, const MinimalSocket::Address& addr2)
    {
        return addr1.getHost() == addr2.getHost() && addr1.getPort() == addr2.getPort();
    }

    template <typename T> T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
    {
        if (node->has_parameter(name))
            return node->get_parameter_or<T>(name, defaultValue);
        else
            return node->declare_parameter<T>(name, defaultValue);
    }

#include <chrono>

    namespace Utils::Time
    {
        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::_V2::system_clock::duration Duration;
        typedef std::chrono::_V2::system_clock::time_point TimePoint;

        inline double toSeconds(Duration duration)
        {
            constexpr double nanoToSec = 1e-9;
            return duration.count() * nanoToSec;
        }

        struct Stopwatch
        {
            Clock clock;
            TimePoint start;

            Stopwatch()
                : clock(), start(clock.now())
            {
            }

            double ellapsed()
            {
                return toSeconds(clock.now() - start);
            }

            void restart()
            {
                start = clock.now();
            }
        };
    } // namespace Utils::Time
} // namespace SocketTransfer