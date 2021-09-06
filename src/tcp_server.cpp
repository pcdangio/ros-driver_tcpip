#include "tcp_server.hpp"

#include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>

using namespace driver_modem;

// CONSTRUCTORS
tcp_server_t::tcp_server_t(boost::asio::io_service& io_service, uint32_t id, std::function<void(uint32_t, boost::asio::ip::tcp::socket*)> connection_callback)
    : m_acceptor(io_service),
      m_id(id)
{
    // Store connection callback.
    tcp_server_t::m_callback_connection = connection_callback;
}
tcp_server_t::~tcp_server_t()
{
    tcp_server_t::stop();
}

// CONTROL
bool tcp_server_t::start(driver_modem_msgs::endpoint& local_endpoint)
{

}
void tcp_server_t::stop()
{

}

// ASIO ACCEPTOR
void tcp_server_t::async_accept()
{
    // Create a new socket for accepting connections.
    tcp_server_t::m_socket = new boost::asio::ip::tcp::socket(tcp_server_t::m_acceptor.get_io_service());

    // Start a new asynchronous accept operaton.
    tcp_server_t::m_acceptor.async_accept(*tcp_server_t::m_socket, boost::bind(&tcp_server_t::accept_callback, this, boost::asio::placeholders::error));
}
void tcp_server_t::accept_callback(const boost::system::error_code &error)
{
    // Check if there was an error.
    if(!error)
    {
        // Raise the connection callback.
        tcp_server_t::m_callback_connection(tcp_server_t::m_id, tcp_server_t::m_socket);

        // This passes the ownership of the socket through the callback.
        tcp_server_t::m_socket = nullptr;
    }
    else
    {
        // An error occured.

        // Clean up socket.
        delete tcp_server_t::m_socket;
        tcp_server_t::m_socket = nullptr;

        // Check if operation was aborted.
        if(error == boost::asio::error::operation_aborted)
        {
            // Quit accept loop.
            return;
        }

        // Report error.
        ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to accept connection (" << error.message() << ")");
    }

    // Restart accept loop.
    tcp_server_t::async_accept();  
}