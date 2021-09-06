#include "tcp_server.hpp"

using namespace driver_modem;

// CONSTRUCTORS
tcp_server_t::tcp_server_t(boost::asio::io_service& io_service, uint32_t id)
    : m_acceptor(io_service),
      m_id(id)
{

}
tcp_server_t::~tcp_server_t()
{

}