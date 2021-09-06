#include "socket.hpp"

using namespace driver_modem;

// CONSTRUCTORS
socket_t::socket_t(uint32_t id, protocol_t protocol)
    : m_id(id),
      m_protocol(protocol)
{

}

// PROPERTIES
protocol_t socket_t::protocol() const
{
    return socket_t::m_protocol;
}