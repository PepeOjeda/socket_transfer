#include "MinimalSocket/core/Definitions.h"
#include "packet.hpp"
#include <istream>
#include <sensor_msgs/msg/compressed_image.hpp>

inline constexpr size_t bufferSize = 10e6;
inline constexpr size_t packetSize = 1500;

// Serialization

template <typename T>
inline void Write(T* address, std::ostream& stream)
{
    stream.write((char*)address, sizeof(T));
}

template <typename T>
inline void Write(T* address, std::ostream& stream, size_t size)
{
    stream.write((char*)address, size);
}

inline std::vector<MinimalSocket::BufferViewConst> Serialize(const sensor_msgs::msg::CompressedImage& msg,
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

    uint16_t numPackets = bytesToWrite / (packetSize - sizeof(PacketHeader));

    // create the stream to write into buffer
    std::stringstream stream;
    stream.rdbuf()->pubsetbuf(mainBufferView.buffer, mainBufferView.buffer_size);

    const uint8_t* dataCurrentPtr = msg.data.data();
    const uint8_t* dataEndPtr = msg.data.data() + msg.data.size();

    // list we will return
    std::vector<MinimalSocket::BufferViewConst> packetViews;

    for (uint16_t packetID = 0; packetID < numPackets; packetID++)
    {
        char* startPtr = mainBufferView.buffer + stream.tellp();

        // write packet
        //-----------------------------------
        PacketHeader packetHeader{.imageID = imageID, .packetID = packetID, .numPackets = numPackets};
        Write(&packetHeader, stream);

        // first packet includes the msg header
        if (packetID == 0)
        {
            // Header
            Write(&msg.header.stamp.sec, stream);
            Write(&msg.header.stamp.nanosec, stream);

            uint16_t frameIDSize = msg.header.frame_id.length();
            Write(&frameIDSize, stream);
            Write(msg.header.frame_id.data(), stream, frameIDSize);

            // format
            uint16_t formatSize = msg.format.length();
            Write(&formatSize, stream);
            Write(msg.format.data(), stream, formatSize);

            // data
            size_t dataSize = msg.data.size();
            Write(&dataSize, stream);
        }

        // write as much of the image data as will fit in the current package, and advance the data pointer accordingly
        char* currentBufferPtr = mainBufferView.buffer + stream.tellp();
        size_t remainingBytesMsgData = dataEndPtr - dataCurrentPtr;
        size_t remainingBytesInPacket = packetSize - (currentBufferPtr - startPtr);

        size_t writeSize = std::min(remainingBytesMsgData, remainingBytesInPacket);
        Write(dataCurrentPtr, stream, writeSize);
        dataCurrentPtr += writeSize;

        // add the bufferview for this packet to the vector
        //-----------------------------------
        currentBufferPtr = mainBufferView.buffer + stream.tellp();
        MinimalSocket::BufferViewConst packetBufView{startPtr, (size_t)(currentBufferPtr - startPtr)};
        packetViews.push_back(packetBufView);
    }
    return packetViews;
}

// Deserialization

template <typename T>
inline void Read(T* address, std::istream& stream)
{
    stream.read((char*)address, sizeof(T));
}

template <typename T>
inline void Read(T* address, std::istream& stream, size_t size)
{
    stream.read((char*)address, size);
}

inline Packet ReadPacketAndAdvance(MinimalSocket::BufferView& bufferView, size_t dataSize)
{
    std::stringstream stream;
    stream.rdbuf()->pubsetbuf(bufferView.buffer, bufferView.buffer_size);

    Packet packet;
    Read(&packet.header, stream);
    packet.data.buffer = bufferView.buffer + sizeof(PacketHeader);
    packet.data.buffer_size = dataSize;

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
        std::stringstream stream;
        stream.rdbuf()->pubsetbuf(packets[0].data.buffer, packets[0].data.buffer_size);

        // Read Image message header from first packet
        //  Header
        Read(&msg.header.stamp.sec, stream);
        Read(&msg.header.stamp.nanosec, stream);

        uint16_t frameIDSizeize;
        Read(&frameIDSizeize, stream);
        msg.header.frame_id.resize(frameIDSizeize);
        Read(msg.header.frame_id.data(), stream, frameIDSizeize);

        // format
        uint16_t formatSize;
        Read(&formatSize, stream);
        msg.format.resize(formatSize);
        Read(msg.format.data(), stream, formatSize);

        // data
        size_t dataSize;
        Read(&dataSize, stream);
        msg.data.resize(dataSize);
        currentDataPtr = msg.data.data();

        Read(msg.data.data(), stream, packets[0].data.buffer_size - stream.tellg());
    }

    for (int i = 1; i < packets.size(); i++)
    {
        const Packet& packet = packets[i];
        std::stringstream stream;
        stream.rdbuf()->pubsetbuf(packet.data.buffer, packet.data.buffer_size);

        Read(currentDataPtr, stream, packet.data.buffer_size);
    }
}
