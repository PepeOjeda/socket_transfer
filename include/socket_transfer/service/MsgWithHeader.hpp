#pragma once
#include <rmw/types.h>
#include <socket_transfer/serialization.hpp>

namespace SocketTransfer
{
    template <typename Msg>
    struct RequestMsg
    {
        rmw_request_id_t header;
        typename Msg::Request request;
    };

    template <typename ServiceT>
    struct Serializer<RequestMsg<ServiceT>>
    {
        static MinimalSocket::BufferView Serialize(const RequestMsg<ServiceT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.header);

            MinimalSocket::BufferView payloadBuffer = Serializer<typename ServiceT::Request>::Serialize(msg.request, writer.getRemainingBuffer());
            return MinimalSocket::BufferView{.buffer = bufferView.buffer, .buffer_size = writer.currentOffset() + payloadBuffer.buffer_size};
        }

        static void Deserialize(RequestMsg<ServiceT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.header);

            Serializer<typename ServiceT::Request>::Deserialize(msg.request, reader.getRemainingBuffer());
        }
    };

    template <typename Msg>
    struct ResponseMsg
    {
        rmw_request_id_t header;
        typename Msg::Response response;
    };

    template <typename ServiceT>
    struct Serializer<ResponseMsg<ServiceT>>
    {
        static MinimalSocket::BufferView Serialize(const ResponseMsg<ServiceT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferWriter writer(bufferView);
            writer.Write(&msg.header);

            MinimalSocket::BufferView payloadBuffer = Serializer<typename ServiceT::Response>::Serialize(msg.response, writer.getRemainingBuffer());
            return MinimalSocket::BufferView{.buffer = bufferView.buffer, .buffer_size = writer.currentOffset() + payloadBuffer.buffer_size};
        }

        static void Deserialize(ResponseMsg<ServiceT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.header);

            Serializer<typename ServiceT::Response>::Deserialize(msg.response, reader.getRemainingBuffer());
        }
    };
} // namespace SocketTransfer