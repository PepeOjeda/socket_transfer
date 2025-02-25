#include "MinimalSocket/core/Definitions.h"
#include "packet.hpp"
#include <cassert>
#include <cmath>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string.h>

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

inline std::vector<Packet> Serialize(const sensor_msgs::msg::CompressedImage& msg,
                                     uint8_t imageID,
                                     MinimalSocket::BufferView mainBufferView)
{
    size_t bytesToWrite =
        sizeof(msg.header.stamp)       //
        + sizeof(uint16_t)             // length of frame id
        + msg.header.frame_id.length() //
        + sizeof(uint16_t)             // length of format string
        + msg.format.length()          //
        + sizeof(size_t)               // length of image data
        + msg.data.size();

    uint16_t numPackets = std::ceil(bytesToWrite / static_cast<double>(packetSize - sizeof(PacketHeader)));

    const uint8_t* dataCurrentPtr = msg.data.data();
    const uint8_t* dataEndPtr = msg.data.data() + msg.data.size();

    BufferWriter writer(mainBufferView.buffer, mainBufferView.buffer_size);

    // list we will return
    std::vector<Packet> packetViews;

    for (uint16_t packetID = 0; packetID < numPackets; packetID++)
    {
        if (packetID == numPackets - 1)
            printf("final package");
        char* headerStartPtr = mainBufferView.buffer + writer.currentOffset();

        // write packet
        //-----------------------------------
        PacketHeader packetHeader{.imageID = imageID, .packetID = packetID, .numPackets = numPackets};
        writer.Write(&packetHeader);

        // first packet includes the msg header
        if (packetID == 0)
        {
            // Header
            writer.Write(&msg.header.stamp.sec);
            writer.Write(&msg.header.stamp.nanosec);

            uint16_t frameIDSize = msg.header.frame_id.length();
            writer.Write(&frameIDSize);
            writer.Write(msg.header.frame_id.data(), frameIDSize);

            // format
            uint16_t formatSize = msg.format.length();
            writer.Write(&formatSize);
            writer.Write(msg.format.data(), formatSize);

            // data
            size_t dataSize = msg.data.size();
            writer.Write(&dataSize);
        }

        // write as much of the image data as will fit in the current package, and advance the data pointer accordingly
        char* currentBufferPtr = mainBufferView.buffer + writer.currentOffset();
        size_t remainingBytesMsgData = dataEndPtr - dataCurrentPtr;
        size_t remainingBytesInPacket = packetSize - (currentBufferPtr - headerStartPtr);

        size_t writeSize = std::min(remainingBytesMsgData, remainingBytesInPacket);
        writer.Write(dataCurrentPtr, writeSize);
        dataCurrentPtr += writeSize;

        // add the bufferview for this packet to the vector
        //-----------------------------------
        currentBufferPtr = mainBufferView.buffer + writer.currentOffset();
        MinimalSocket::BufferView packetBufView{headerStartPtr, (size_t)(currentBufferPtr - headerStartPtr)};
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

inline void Deserialize(sensor_msgs::msg::CompressedImage& msg, const std::vector<Packet>& packets)
{
    uint8_t* currentDataPtr; // ptr to the next position to fill inside of the image msg

    // parse first packet, which contains the message header
    {
        BufferReader reader(packets[0].data.buffer, packets[0].data.buffer_size);

        // Read Image message header from first packet
        //  Header
        reader.Read(&msg.header.stamp.sec);
        reader.Read(&msg.header.stamp.nanosec);

        uint16_t frameIDSize;
        reader.Read(&frameIDSize);
        msg.header.frame_id.resize(frameIDSize);
        reader.Read(msg.header.frame_id.data(), frameIDSize);

        // format
        uint16_t formatSize;
        reader.Read(&formatSize);
        msg.format.resize(formatSize);
        reader.Read(msg.format.data(), formatSize);

        // data
        size_t dataSize;
        reader.Read(&dataSize);
        msg.data.resize(dataSize);
        currentDataPtr = msg.data.data();

        size_t dataThisPacket = packets[0].data.buffer_size - reader.currentOffset();
        reader.Read(msg.data.data(), dataThisPacket);
        currentDataPtr += dataThisPacket;
    }

    for (int i = 1; i < packets.size(); i++)
    {
        const Packet& packet = packets[i];
        BufferReader reader(packet.data.buffer, packet.data.buffer_size);
        reader.Read(currentDataPtr, packet.data.buffer_size);
        currentDataPtr += packet.data.buffer_size;
    }
}

inline void RelocatePacket(Packet& packet, MinimalSocket::BufferView& bufferView)
{
    BufferWriter writer(bufferView.buffer, bufferView.buffer_size);
    writer.Write(packet.data.buffer, packet.data.buffer_size);

    packet.data.buffer = bufferView.buffer;
    
    bufferView.buffer += writer.currentOffset();
    bufferView.buffer_size -= writer.currentOffset();
}