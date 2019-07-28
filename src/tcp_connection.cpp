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
}
tcp_connection::~tcp_connection()
{
    delete [] tcp_connection::m_buffer;
}

// METHODS
bool tcp_connection::connect()
{
    try
    {
        // Start connection.
        tcp_connection::m_socket.connect(tcp_connection::m_remote_endpoint);

        // Start first asynchronous read.
        tcp_connection::async_rx();

        return true;
    }
    catch (...)
    {
        return false;
    }
}
void tcp_connection::attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback)
{
    tcp_connection::m_rx_callback = callback;
}
void tcp_connection::tx(uint8_t *data, uint32_t length)
{
    tcp_connection::m_socket.send(boost::asio::buffer(data, length));

    delete [] data;
}
void tcp_connection::async_rx()
{
    tcp_connection::m_socket.async_receive(boost::asio::buffer(tcp_connection::m_buffer, tcp_connection::m_buffer_size),
                                                boost::bind(&tcp_connection::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

// CALLBACKS
void tcp_connection::rx_callback(const boost::system::error_code &error, std::size_t bytes_read)
{
    // Make sure there are no errors, and that the rx callback is attached.
    if(!error && tcp_connection::m_rx_callback)
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
