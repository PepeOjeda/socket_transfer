#pragma once
#include "BufferUtils.hpp"
#include <MinimalSocket/core/Definitions.h>
#include <cassert>
#include <cmath>
#include <socket_transfer/internals/packet.hpp>

namespace SocketTransfer
{
    inline constexpr size_t bufferSize = 10e6;
    inline constexpr size_t packetSize = 1500;

    // must be specialized for each type of message
    template <typename T>
    struct Serializer;
    // In this specialization, you must define two static methods:

    // static MinimalSocket::BufferView Serialize(const T& msg, MinimalSocket::BufferView bufferView)
    //      returns a BufferView that points to the serialized message, with the size that was used (not the size that is available in the entire buffer)
    //      the original BufferView is unchanged, the caller is responsible for advancing the pointer if required

    // static void Deserialize(T& msg, MinimalSocket::BufferView bufferView)
    //      expects the bufferView to have the correct size
    //      the original BufferView is unchanged, the caller is responsible for advancing the pointer if required

    // Serialization
    //-----------------------------------------------------
    //-----------------------------------------------------

    // the data field of the packets returned by this includes the packet header itself, even if a copy of the header exists separately from it
    // reasoning being, it makes the process of sending the message easier by having a single bufferview include both header and data
    inline std::vector<Internal::Packet>
    DividePackets(MinimalSocket::BufferView source,
                  uint8_t messageID,
                  MinimalSocket::BufferView destination)
    {
        size_t bytesToWrite = source.buffer_size;

        uint16_t numPackets = std::ceil(bytesToWrite / static_cast<double>(packetSize - sizeof(Internal::PacketHeader)));

        const char* dataCurrentPtr = source.buffer;
        const char* dataEndPtr = source.buffer + source.buffer_size;

        BufferWriter writer(destination.buffer, destination.buffer_size);

        // list we will return
        std::vector<Internal::Packet> packetViews;

        for (uint16_t packetID = 0; packetID < numPackets; packetID++)
        {
            if (packetID == numPackets - 1)
                printf("final package");
            char* packetStartPtr = destination.buffer + writer.currentOffset();

            // write as much of the message data as will fit in the current package
            size_t remainingBytesInPacket = packetSize - sizeof(Internal::PacketHeader);
            size_t remainingBytesMsgData = dataEndPtr - dataCurrentPtr;

            size_t writeSize = std::min(remainingBytesMsgData, remainingBytesInPacket);

            // write packet
            //-----------------------------------
            Internal::PacketHeader packetHeader{
                .packetSize = (uint16_t)(writeSize + sizeof(packetHeader)),
                .msgType = Internal::PacketHeader::MsgType::Data,
                .messageID = messageID,
                .packetID = packetID,
                .numPackets = numPackets};
            writer.Write(&packetHeader);

            writer.Write(dataCurrentPtr, writeSize);
            dataCurrentPtr += writeSize;

            // add the bufferview for this packet to the vector
            //-----------------------------------
            char* currentBufferPtr = destination.buffer + writer.currentOffset();
            MinimalSocket::BufferView packetBufView{packetStartPtr, (size_t)(currentBufferPtr - packetStartPtr)};
            packetViews.push_back(Internal::Packet{packetHeader, packetBufView});
        }
        return packetViews;
    }

    // Deserialization
    //-----------------------------------------------------
    //-----------------------------------------------------

    inline Internal::Packet ReadPacketAndAdvance(MinimalSocket::BufferView& bufferView, size_t dataSize)
    {
        BufferReader reader(bufferView.buffer, bufferView.buffer_size);

        Internal::Packet packet;
        reader.Read(&packet.header);
        packet.data.buffer = bufferView.buffer + sizeof(Internal::PacketHeader);
        packet.data.buffer_size = dataSize - sizeof(Internal::PacketHeader);

        // update bufferview
        bufferView.buffer += dataSize;
        bufferView.buffer_size -= dataSize;

        return packet;
    }

    // moves all of this msg's data into a buffer. Contiguously, in order, and without packet headers
    // the original BufferView is unchanged, the caller is responsible for advancing the pointer if required
    inline MinimalSocket::BufferView ExtractData(const std::vector<Internal::Packet>& packets, MinimalSocket::BufferView destination)
    {
        MinimalSocket::BufferView extractedData = destination;
        for (int i = 0; i < packets.size(); i++)
        {
            const Internal::Packet& packet = packets[i];
            BufferReader reader(packet.data.buffer, packet.data.buffer_size);
            reader.Read(destination.buffer, packet.data.buffer_size);
            destination.buffer += packet.data.buffer_size;
        }
        // how much has the pointer advanced while reading into the buffer?
        extractedData.buffer_size = destination.buffer - extractedData.buffer;
        return extractedData;
    }

    inline void RelocatePacket(Internal::Packet& packet, MinimalSocket::BufferView& bufferView)
    {
        BufferWriter writer(bufferView.buffer, bufferView.buffer_size);
        writer.Write(packet.data.buffer, packet.data.buffer_size);

        packet.data.buffer = bufferView.buffer;

        bufferView.buffer += writer.currentOffset();
        bufferView.buffer_size -= writer.currentOffset();
    }
} // namespace SocketTransfer

#include <std_msgs/msg/header.hpp>
namespace SocketTransfer::SerializationUtils
{
    inline void SerializeString(BufferWriter& writer, const std::string& str)
    {
        size_t length = str.length();
        writer.Write(&length);
        writer.Write(str.data(), length);
    }

    inline void DeserializeString(BufferReader& reader, std::string& str)
    {
        size_t length = str.length();
        reader.Read(&length);
        str.resize(length);
        reader.Read(str.data(), length);
    }

    inline void SerializeHeader(BufferWriter& writer, const std_msgs::msg::Header& header)
    {
        writer.Write(&header.stamp);
        SerializeString(writer, header.frame_id);
    }

    inline void DeserializeHeader(BufferReader& reader, std_msgs::msg::Header& header)
    {
        reader.Read(&header.stamp);
        DeserializeString(reader, header.frame_id);
    }

    template <typename T>
    inline void SerializeVector(BufferWriter& writer, const std::vector<T>& vec)
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Type is not trivially copyable! If your type contains dynamic memory (strings, vectors, etc) you must explicitly define how to serialize it");
        size_t length = vec.size();
        writer.Write(&length);
        writer.Write(vec.data(), length * sizeof(T));
    }

    template <typename T>
    inline void DeserializeVector(BufferReader& reader, std::vector<T>& vec)
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Type is not trivially copyable! If your type contains dynamic memory (strings, vectors, etc) you must explicitly define how to serialize it");
        size_t length;
        reader.Read(&length);
        vec.resize(length);
        reader.Read(vec.data(), length * sizeof(T));
    }


    // specialization for vector of strings, which is allowed in ROS interfaces
    template <>
    inline void SerializeVector(BufferWriter& writer, const std::vector<std::string>& vec)
    {
        size_t length = vec.size();
        writer.Write(&length);
        for (const std::string& str : vec)
        {
            SerializeString(writer, str);
        }
    }

    template <>
    inline void DeserializeVector(BufferReader& reader, std::vector<std::string>& vec)
    {
        size_t length;
        reader.Read(&length);
        vec.resize(length);
        for (std::string& str : vec)
        {
            DeserializeString(reader, str);
        }
    }
} // namespace SocketTransfer::SerializationUtils