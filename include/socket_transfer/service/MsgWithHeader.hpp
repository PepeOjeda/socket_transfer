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

            return Serializer<typename ServiceT::Request>::Serialize(msg.request, writer.getRemainingBuffer());
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

            return Serializer<typename ServiceT::Response>::Serialize(msg.response, writer.getRemainingBuffer());
        }

        static void Deserialize(ResponseMsg<ServiceT>& msg, MinimalSocket::BufferView bufferView)
        {
            BufferReader reader(bufferView);
            reader.Read(&msg.header);

            Serializer<typename ServiceT::Response>::Deserialize(msg.response, reader.getRemainingBuffer());
        }
    };
} // namespace SocketTransfer