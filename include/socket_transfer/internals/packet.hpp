#include "MinimalSocket/core/Definitions.h"
#include <algorithm>
#include <cstdint>
#include <vector>

namespace SocketTransfer
{
    struct PacketHeader
    {
        uint16_t packetSize;
        uint8_t messageID;
        uint16_t packetID;
        uint16_t numPackets;
    };

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
        }

        void printmessageIDError()
        {
            fprintf(stderr, "Message IDs are not all the same! Look:\n");
            for (int j = 0; j < packets.size(); j++)
                fprintf(stderr, "\t %d\n", packets[j].header.messageID);
        }
    };
} // namespace SocketTransfer