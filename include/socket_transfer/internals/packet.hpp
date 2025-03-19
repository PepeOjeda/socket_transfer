#include "../BufferUtils.hpp"
#include "MinimalSocket/core/Definitions.h"
#include "signal.h"
#include <algorithm>
#include <cstdint>
#include <ostream>
#include <vector>

namespace SocketTransfer::Internal
{
    struct PacketHeader
    {
        enum class MsgType : uint8_t
        {
            Hi,
            HiOK,
            Bye,
            Data
        };

        uint16_t packetSize;
        MsgType msgType;
        uint8_t messageID;
        uint16_t packetID;
        uint16_t numPackets;
    };

    inline PacketHeader ReadHeader(MinimalSocket::BufferView bufView)
    {
        BufferReader reader(bufView.buffer, bufView.buffer_size);

        PacketHeader packet;
        reader.Read(&packet);
        return packet;
    }

    struct Packet
    {
        PacketHeader header;
        MinimalSocket::BufferView data;
    };

    struct Message
    {
        std::vector<Packet> packets;

        bool isComplete()
        {
            if (packets.size() == 0 || packets.size() < packets.at(0).header.numPackets)
                return false;
            sort();

            uint8_t messageID = packets.at(0).header.messageID;
            // check if we have duplicates or something, i don't know
            for (int i = 0; i < packets.size(); i++)
            {
                if (packets[i].header.messageID != messageID)
                {
                    printmessageIDError();
                    return false;
                }
                if (packets[i].header.packetID != i)
                {
                    printPacketIDError();
                    return false;
                }
            }

            return true;
        }

        uint16_t messageID()
        {
            if (packets.size() == 0)
            {
                fprintf(stderr, "Asked for message ID but no packets exist!");
                return 0xffff;
            }
            return packets[0].header.messageID;
        }

        void sort()
        {
            std::sort(packets.begin(), packets.end(), [](const Packet& a, const Packet& b)
                      {
                          return a.header.packetID < b.header.packetID;
                      });
        }

        void printPacketIDError()
        {
            fprintf(stderr, "Packet IDs in message %d are weird! Look:\n", packets[0].header.messageID);
            for (int j = 0; j < packets.size(); j++)
                fprintf(stderr, "\t %d\n", packets[j].header.packetID);
            raise(SIGTRAP);
        }

        void printmessageIDError()
        {
            fprintf(stderr, "Message IDs are not all the same! Look:\n");
            for (int j = 0; j < packets.size(); j++)
                fprintf(stderr, "\t %d\n", packets[j].header.messageID);
            raise(SIGTRAP);
        }
    };

    inline const char* typeAsStr(PacketHeader::MsgType type)
    {
        switch (type)
        {
        case PacketHeader::MsgType::Hi:
            return "HI";
        case PacketHeader::MsgType::HiOK:
            return "HIOK";
        case PacketHeader::MsgType::Bye:
            return "BYE";
        case PacketHeader::MsgType::Data:
            return "Data";
        }
        return "";
    }

    inline std::ostream& operator<<(std::ostream& stream, const PacketHeader& header)
    {
        stream << "\n\tsize: " << (uint)header.packetSize;
        stream << "\n\tmsgType: " << typeAsStr(header.msgType);
        stream << "\n\tmessageID: " << (uint)header.messageID;
        stream << "\n\tpacketID: " << (uint)header.packetID;
        stream << "\n\tnumPackets: " << (uint)header.numPackets << "\n";
        return stream;
    }
} // namespace SocketTransfer::Internal