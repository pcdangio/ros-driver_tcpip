#include "tcp_connection.h"

#include <boost/bind.hpp>

// CONSTRUCTORS
tcp_connection::tcp_connection(boost::asio::io_service& io_service, tcp::endpoint local_endpoint, tcp::endpoint remote_endpoint, uint32_t buffer_size)
    // Initialize socket and acceptor
    : m_socket(io_service, local_endpoint),
      m_acceptor(io_service, local_endpoint)
{
    // Store remote endpoint.
    tcp_connection::m_remote_endpoint = remote_endpoint;

    // Dynamically allocate buffer.
    tcp_connection::m_buffer_size = buffer_size;
    tcp_connection::m_buffer = new uint8_t[buffer_size];

    // Set internal connection flag.
    tcp_connection::m_status = tcp_connection::status::DISCONNECTED;
}
tcp_connection::~tcp_connection()
{
    delete [] tcp_connection::m_buffer;
}

// METHODS
tcp_connection::status tcp_connection::connect(tcp_role connection_role)
{
    if(tcp_connection::m_status == tcp_connection::status::DISCONNECTED)
    {
        switch(connection_role)
        {
        case tcp_role::SERVER:
        {
            // Start asynchronously accepting connections.
            tcp_connection::async_accept();

            // Update connection status flag.
            tcp_connection::m_status = tcp_connection::status::PENDING;

            break;
        }
        case tcp_role::CLIENT:
        {
            try
            {
                // Start connection.
                tcp_connection::m_socket.connect(tcp_connection::m_remote_endpoint);

                // Update connection status flag.
                tcp_connection::m_status = tcp_connection::status::CONNECTED;

                // Start first asynchronous read.
                tcp_connection::async_rx();
            }
            catch (...)
            {
                // Update connection status flag.
                tcp_connection::m_status = tcp_connection::status::DISCONNECTED;
            }

            break;
        }
        }
    }

    return tcp_connection::m_status;
}
void tcp_connection::attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback)
{
    tcp_connection::m_rx_callback = callback;
}
void tcp_connection::attach_disconnect_callback(std::function<void(uint16_t)> callback)
{
    tcp_connection::m_disconnect_callback = callback;
}
bool tcp_connection::tx(const uint8_t *data, uint32_t length)
{
    // Check if connection is active.
    if(tcp_connection::m_status == tcp_connection::status::CONNECTED)
    {
        // Send message with error reporting.
        boost::system::error_code error;
        tcp_connection::m_socket.send(boost::asio::buffer(data, length), 0, error);

        // Check if error is broken_pipe, indicating connection broken.
        if(error.value() == boost::system::errc::broken_pipe)
        {
            // Connection is broken.
            tcp_connection::m_status = tcp_connection::status::DISCONNECTED;
            if(tcp_connection::m_disconnect_callback)
            {
                tcp_connection::m_disconnect_callback(tcp_connection::m_socket.local_endpoint().port());
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}
void tcp_connection::async_accept()
{
    tcp_connection::m_acceptor.async_accept(tcp_connection::m_socket, boost::bind(&tcp_connection::accept_callback, this, boost::placeholders::_1));
}
void tcp_connection::async_rx()
{
    tcp_connection::m_socket.async_receive(boost::asio::buffer(tcp_connection::m_buffer, tcp_connection::m_buffer_size),
                                           boost::bind(&tcp_connection::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

// PROPERTIES
tcp_connection::status tcp_connection::p_status() const
{
    return tcp_connection::m_status;
}

// CALLBACKS
void tcp_connection::accept_callback(const boost::system::error_code &error)
{
    if(!error)
    {
        // Connection has been made.

        // Update connection status flag.
        tcp_connection::m_status = tcp_connection::status::CONNECTED;

        // Raise connected callback to signal external code.
        if(tcp_connection::m_connect_callback)
        {
            tcp_connection::m_connect_callback(tcp_connection::m_socket.local_endpoint().port());
        }

        // Start first asynchronous read.
        tcp_connection::async_rx();
    }
    else
    {
        // Restart async_accept
        tcp_connection::async_accept();
    }
}
void tcp_connection::rx_callback(const boost::system::error_code &error, std::size_t bytes_read)
{
    // Make sure there are no errors, and that the rx callback is attached.
    if(!error)
    {
        if(tcp_connection::m_rx_callback)
        {
            // Deep copy the data into a new output array.
            uint8_t* output_array = new uint8_t[bytes_read];
            std::memcpy(output_array, tcp_connection::m_buffer, bytes_read);

            // Raise the callback.
            tcp_connection::m_rx_callback(connection_type::TCP, tcp_connection::m_socket.local_endpoint().port(), output_array, static_cast<uint32_t>(bytes_read));
        }

        // Start a new asynchronous receive.
        tcp_connection::async_rx();
    }
    else if(error == boost::asio::error::eof || error == boost::asio::error::connection_reset || error == boost::asio::error::connection_aborted)
    {
        // Connection has been closed.
        tcp_connection::m_status = tcp_connection::status::DISCONNECTED;
        if(tcp_connection::m_disconnect_callback)
        {
            tcp_connection::m_disconnect_callback(tcp_connection::m_socket.local_endpoint().port());
        }
    }
}
