#include "MinimalSocket/core/Definitions.h"
#include <algorithm>
#include <cstdint>
#include <vector>

struct PacketHeader
{
    uint8_t imageID;
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

        // check if we have duplicates or something, i don't know
        for (int i = 0; i < packets.size(); i++)
            if (packets[i].header.packetID != i)
            {
                printError();
                return false;
            }

        return true;
    }

    uint16_t imageID()
    {
        if (packets.size() == 0)
        {
            fprintf(stderr, "Asked for image ID but no packets exist!");
            return 0xffff;
        }
        return packets[0].header.imageID;
    }

    void sort()
    {
        std::sort(packets.begin(), packets.end(), [](const Packet& a, const Packet& b)
                  {
                      return a.header.packetID < b.header.packetID;
                  });
    }

    void printError()
    {
        fprintf(stderr, "Packet IDs in message %d are weird! Look:\n", packets[0].header.imageID);
        for (int j = 0; j < packets.size(); j++)
            fprintf(stderr, "\t %d\n", packets[j].header.packetID);
    }
};