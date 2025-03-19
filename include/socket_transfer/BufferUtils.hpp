#pragma once
#include "MinimalSocket/core/Definitions.h"
#include <string.h>


class BufferWriter
{
public:
    BufferWriter() = delete;
    BufferWriter(char* _start, size_t _size)
    {
        start = _start;
        current = start;
        end = start + _size;
    }

    BufferWriter(MinimalSocket::BufferView bufferView)
        : BufferWriter(bufferView.buffer, bufferView.buffer_size)
    {}

    template <typename T>
    void Write(T* address)
    {
        Write(address, sizeof(T));
    }

    template <typename T>
    void Write(T* address, size_t size)
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Type is not trivially copyable! If your type contains dynamic memory (strings, vectors, etc) you must explicitly define how to serialize it");
        memcpy(current, address, size);
        current += size;
    }

    size_t currentOffset()
    {
        return current - start;
    }

    MinimalSocket::BufferView getRemainingBuffer()
    {
        return MinimalSocket::BufferView{.buffer = current, .buffer_size = (size_t)(end - current)};
    }

    MinimalSocket::BufferView getUsedBufferView()
    {
        return MinimalSocket::BufferView{.buffer = start, .buffer_size = (size_t)(current - start)};
    }

private:
    char* start;
    char* current;
    char* end;
};

class BufferReader
{
public:
    BufferReader() = delete;
    BufferReader(char* _start, size_t _size)
    {
        start = _start;
        current = start;
        end = start + _size;
    }

    BufferReader(MinimalSocket::BufferView bufferView)
        : BufferReader(bufferView.buffer, bufferView.buffer_size)
    {}

    template <typename T>
    void Read(T* address)
    {
        Read(address, sizeof(T));
    }

    template <typename T>
    void Read(T* address, size_t size)
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Type is not trivially copyable! If your type contains dynamic memory (strings, vectors, etc) you must explicitly define how to serialize it");
        memcpy(address, current, size);
        current += size;
    }

    size_t currentOffset()
    {
        return current - start;
    }

    MinimalSocket::BufferView getRemainingBuffer()
    {
        return MinimalSocket::BufferView{.buffer = current, .buffer_size = (size_t)(end - current)};
    }

private:
    char* start;
    char* current;
    char* end;
};