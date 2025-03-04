#pragma once
#include "MinimalSocket/core/Definitions.h"
#include "packet.hpp"
#include <cassert>
#include <cmath>
#include <string.h>

namespace SocketTransfer
{
    inline constexpr size_t bufferSize = 10e6;
    inline constexpr size_t packetSize = 1500;

    // Serialization

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

        template <typename T>
        void Write(T* address)
        {
            memcpy(current, address, sizeof(T));
            current += sizeof(T);
        }

        template <typename T>
        void Write(T* address, size_t size)
        {
            memcpy(current, address, size);
            current += size;
        }

        size_t currentOffset()
        {
            return current - start;
        }

    private:
        char* start;
        char* current;
        char* end;
    };

    // must be specialized for each type of message
    // returns a BufferView that points to the serialized message, with the size that was used (not the size that is available in the entire buffer)
    // the original BufferView is unchanged, the caller is responsible for advancing the pointer if required
    template <typename T>
    MinimalSocket::BufferView Serialize(const T& msg, MinimalSocket::BufferView bufferView);

    // the data field of the packets returned by this includes the packet header itself, even if a copy of the header exists separately from it
    // reasoning being, it makes the process of sending the message easier by having a single bufferview include both header and data
    inline std::vector<Packet> DividePackets(MinimalSocket::BufferView source,
                                             uint8_t messageID,
                                             MinimalSocket::BufferView destination)
    {
        size_t bytesToWrite = source.buffer_size;

        uint16_t numPackets = std::ceil(bytesToWrite / static_cast<double>(packetSize - sizeof(PacketHeader)));

        const char* dataCurrentPtr = source.buffer;
        const char* dataEndPtr = source.buffer + source.buffer_size;

        BufferWriter writer(destination.buffer, destination.buffer_size);

        // list we will return
        std::vector<Packet> packetViews;

        for (uint16_t packetID = 0; packetID < numPackets; packetID++)
        {
            if (packetID == numPackets - 1)
                printf("final package");
            char* packetStartPtr = destination.buffer + writer.currentOffset();

            // write packet
            //-----------------------------------
            PacketHeader packetHeader{.messageID = messageID, .packetID = packetID, .numPackets = numPackets};
            writer.Write(&packetHeader);

            // write as much of the message data as will fit in the current package, and advance the data pointer accordingly
            char* currentBufferPtr = destination.buffer + writer.currentOffset();
            size_t remainingBytesMsgData = dataEndPtr - dataCurrentPtr;
            size_t remainingBytesInPacket = packetSize - (currentBufferPtr - packetStartPtr);

            size_t writeSize = std::min(remainingBytesMsgData, remainingBytesInPacket);
            writer.Write(dataCurrentPtr, writeSize);
            dataCurrentPtr += writeSize;

            // add the bufferview for this packet to the vector
            //-----------------------------------
            currentBufferPtr = destination.buffer + writer.currentOffset();
            MinimalSocket::BufferView packetBufView{packetStartPtr, (size_t)(currentBufferPtr - packetStartPtr)};
            packetViews.push_back(Packet{packetHeader, packetBufView});
        }
        return packetViews;
    }

    // Deserialization

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

        template <typename T>
        void Read(T* address)
        {
            memcpy(address, current, sizeof(T));
            current += sizeof(T);
        }

        template <typename T>
        void Read(T* address, size_t size)
        {
            memcpy(address, current, size);
            current += size;
        }

        size_t currentOffset()
        {
            return current - start;
        }

    private:
        char* start;
        char* current;
        char* end;
    };

    template <typename T>
    void Deserialize(T& msg, MinimalSocket::BufferView bufferView);

    inline Packet ReadPacketAndAdvance(MinimalSocket::BufferView& bufferView, size_t dataSize)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        Packet packet;
        reader.Read(&packet.header);
        packet.data.buffer = bufferView.buffer + sizeof(PacketHeader);
        packet.data.buffer_size = dataSize - sizeof(PacketHeader);

        // update bufferview
        bufferView.buffer += dataSize;
        bufferView.buffer_size -= dataSize;

        return packet;
    }

    // moves all of this msg's data into a buffer. Contiguously, in order, and without packet headers
    // the original BufferView is unchanged, the caller is responsible for advancing the pointer if required
    inline MinimalSocket::BufferView ExtractData(const std::vector<Packet>& packets, MinimalSocket::BufferView destination)
    {
        MinimalSocket::BufferView extractedData = destination;
        for (int i = 0; i < packets.size(); i++)
        {
            const Packet& packet = packets[i];
            BufferReader reader(packet.data.buffer, packet.data.buffer_size);
            reader.Read(destination.buffer, packet.data.buffer_size);
            destination.buffer += packet.data.buffer_size;
        }
        // how much has the pointer advanced while reading into the buffer?
        extractedData.buffer_size = destination.buffer - extractedData.buffer;
        return extractedData;
    }

    inline void RelocatePacket(Packet& packet, MinimalSocket::BufferView& bufferView)
    {
        BufferWriter writer(bufferView.buffer, bufferView.buffer_size);
        writer.Write(packet.data.buffer, packet.data.buffer_size);

        packet.data.buffer = bufferView.buffer;

        bufferView.buffer += writer.currentOffset();
        bufferView.buffer_size -= writer.currentOffset();
    }
} // namespace SocketTransfer