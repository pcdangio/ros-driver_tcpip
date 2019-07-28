#include "tcp_connection.h"

#include <boost/bind.hpp>
#include <boost/shared_array.hpp>

// CONSTRUCTORS
tcp_connection::tcp_connection(boost::asio::io_service& io_service, tcp::endpoint local_endpoint, tcp::endpoint remote_endpoint, uint32_t buffer_size)
    // Initialize socket.
    :m_socket(io_service, local_endpoint)
{
    // Store remote endpoint.
    tcp_connection::m_remote_endpoint = remote_endpoint;

    // Dynamically allocate buffer.
    tcp_connection::m_buffer_size = buffer_size;
    tcp_connection::m_buffer = new uint8_t[buffer_size];

    // Set internal connection flag.
    tcp_connection::m_connected = false;
}
tcp_connection::~tcp_connection()
{
    delete [] tcp_connection::m_buffer;
}

// METHODS
bool tcp_connection::connect()
{
    if(tcp_connection::m_connected == false)
    {
        try
        {
            // Start connection.
            tcp_connection::m_socket.connect(tcp_connection::m_remote_endpoint);

            // Update connection flag.
            tcp_connection::m_connected = true;

            // Start first asynchronous read.
            tcp_connection::async_rx();

            return true;
        }
        catch (...)
        {
            // Update connection flag.
            tcp_connection::m_connected = false;

            return false;
        }
    }
    else
    {
        return true;
    }
}
void tcp_connection::attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback)
{
    tcp_connection::m_rx_callback = callback;
}
void tcp_connection::attach_disconnect_callback(std::function<void(uint16_t)> callback)
{
    tcp_connection::m_disconnect_callback = callback;
}
bool tcp_connection::tx(uint8_t *data, uint32_t length)
{
    // Check if connection is active.
    if(tcp_connection::m_connected == true)
    {
        // Send message with error reporting.
        boost::system::error_code error;
        tcp_connection::m_socket.send(boost::asio::buffer(data, length), 0, error);

        // Check if error is broken_pipe, indicating connection broken.
        if(error.value() == boost::system::errc::broken_pipe)
        {
            // Connection is broken.
            tcp_connection::m_connected = false;
            if(tcp_connection::m_disconnect_callback)
            {
                tcp_connection::m_disconnect_callback(tcp_connection::m_socket.local_endpoint().port());
            }
        }

        // Clean up the data.
        delete [] data;

        return true;
    }
    else
    {
        return false;
    }
}
void tcp_connection::async_rx()
{
    tcp_connection::m_socket.async_receive(boost::asio::buffer(tcp_connection::m_buffer, tcp_connection::m_buffer_size),
                                                boost::bind(&tcp_connection::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

// CALLBACKS
#include <iostream>
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
        tcp_connection::m_connected = false;
        if(tcp_connection::m_disconnect_callback)
        {
            tcp_connection::m_disconnect_callback(tcp_connection::m_socket.local_endpoint().port());
        }
    }
}
