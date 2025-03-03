#pragma once
#include <rmw/types.h>

namespace SocketTransfer
{
    template <typename Msg>
    struct Request
    {
        rmw_request_id_t header;
        typename Msg::Request request;
    };

    template <typename Msg>
    struct Response
    {
        rmw_request_id_t header;
        typename Msg::Response response;
    };
} // namespace SocketTransfer