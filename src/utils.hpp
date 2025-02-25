#pragma once
#include "MinimalSocket/core/Definitions.h"

inline MinimalSocket::BufferViewConst AsConst(MinimalSocket::BufferView view)
{
    return {.buffer = view.buffer, .buffer_size = view.buffer_size};
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