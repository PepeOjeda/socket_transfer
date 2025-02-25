#include "MinimalSocket/core/Definitions.h"

inline MinimalSocket::BufferViewConst AsConst(MinimalSocket::BufferView view)
{
    return {.buffer = view.buffer, .buffer_size = view.buffer_size};
}