#include "tcp_connection.h"

#include <boost/bind.hpp>

// CONSTRUCTORS
tcp_connection::tcp_connection(boost::asio::io_service& io_service, tcp::endpoint local_endpoint, uint32_t buffer_size)
    // Initialize socket and acceptor
    : m_socket(io_service),
      m_acceptor(io_service)
{
    // Store local endpoint for feeding socket (client) and acceptor (server) connections.
    tcp_connection::m_local_endpoint = local_endpoint;

    // Dynamically allocate buffer.
    tcp_connection::m_buffer_size = buffer_size;
    tcp_connection::m_buffer = new uint8_t[buffer_size];

    // Initialize role and status.
    tcp_connection::m_role = tcp_connection::role::UNASSIGNED;
    tcp_connection::m_status = tcp_connection::status::DISCONNECTED;
}
tcp_connection::~tcp_connection()
{
    delete [] tcp_connection::m_buffer;
}

// METHODS
bool tcp_connection::start_server()
{
    if(tcp_connection::m_status == tcp_connection::status::DISCONNECTED)
    {
        // Open the acceptor
        tcp_connection::m_acceptor.open(tcp_connection::m_local_endpoint.protocol());

        // Bind it to the local endpoint.
        tcp_connection::m_acceptor.bind(tcp_connection::m_local_endpoint);

        // Instruct acceptor to listen on local endpoint.
        tcp_connection::m_acceptor.listen();

        // Start asynchronously accepting connections.
        tcp_connection::async_accept();

        // Set role.
        tcp_connection::m_role = tcp_connection::role::SERVER;

        // Update status.
        tcp_connection::update_status(tcp_connection::status::PENDING);

        return true;
    }
    else
    {
        return false;
    }
}
bool tcp_connection::start_client(tcp::endpoint remote_endpoint)
{
    if(tcp_connection::m_status == tcp_connection::status::DISCONNECTED)
    {
        try
        {
            // Open the socket.
            tcp_connection::m_socket.open(tcp_connection::m_local_endpoint.protocol());

            // Bind socket to the local endpoint.
            tcp_connection::m_socket.bind(tcp_connection::m_local_endpoint);

            // Start async connect attempt.
            tcp_connection::m_socket.async_connect(remote_endpoint, boost::bind(&tcp_connection::connect_callback, this, boost::placeholders::_1));

            // Set role.
            tcp_connection::m_role = tcp_connection::role::CLIENT;

            // Update status.
            tcp_connection::update_status(tcp_connection::status::PENDING);

            return true;
        }
        catch (...)
        {
            // Update status.
            tcp_connection::update_status(tcp_connection::status::DISCONNECTED);

            return false;
        }
    }
    else
    {
        return false;
    }
}
void tcp_connection::disconnect()
{
    // Close the socket
    tcp_connection::m_socket.close();

    // Update status (and ultimately self-delete)
    // Do not raise signal, since this function is called externally.
    tcp_connection::update_status(tcp_connection::status::DISCONNECTED, false);
}
void tcp_connection::attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback)
{
    tcp_connection::m_rx_callback = callback;
}
void tcp_connection::attach_connected_callback(std::function<void (uint16_t)> callback)
{
    tcp_connection::m_connected_callback = callback;
}
void tcp_connection::attach_disconnected_callback(std::function<void(uint16_t)> callback)
{
    tcp_connection::m_disconnected_callback = callback;
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
            // Connection is broken.  Update status.
            tcp_connection::update_status(tcp_connection::status::DISCONNECTED);
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
void tcp_connection::update_status(status new_status, bool signal)
{
    if(new_status != tcp_connection::m_status)
    {
        // Update status flag.
        tcp_connection::m_status = new_status;

        switch(new_status)
        {
        case tcp_connection::status::DISCONNECTED:
        {
            // Reset role to unassigned.
            tcp_connection::m_role = tcp_connection::role::UNASSIGNED;

            // Raise disconnect callback.
            if(signal && tcp_connection::m_disconnected_callback)
            {
                tcp_connection::m_disconnected_callback(tcp_connection::m_socket.local_endpoint().port());
            }

            // Self delete.  This is needed because callbacks can't delete the caller safely.
            delete this;

            break;
        }
        case tcp_connection::status::CONNECTED:
        {
            // Raise connected handler.
            if(signal && tcp_connection::m_connected_callback)
            {
                tcp_connection::m_connected_callback(tcp_connection::m_socket.local_endpoint().port());
            }

            break;
        }
        case tcp_connection::status::PENDING:
        {
            // Do nothing else specific.
            break;
        }
        }
    }

}

// PROPERTIES
tcp_connection::role tcp_connection::p_role() const
{
    return tcp_connection::m_role;
}
tcp_connection::status tcp_connection::p_status() const
{
    return tcp_connection::m_status;
}
tcp::endpoint tcp_connection::p_remote_endpoint() const
{
    return tcp_connection::m_socket.remote_endpoint();
}

// CALLBACKS
void tcp_connection::connect_callback(const boost::system::error_code &error)
{
    if(!error)
    {
        // Client has successfully connected to a server.  Update status.
        tcp_connection::update_status(tcp_connection::status::CONNECTED);

        // Start first asynchronous read.
        tcp_connection::async_rx();
    }
    else
    {
        // Connection failed.  Update status.
        tcp_connection::update_status(tcp_connection::status::DISCONNECTED);
    }
}
void tcp_connection::accept_callback(const boost::system::error_code &error)
{
    if(!error)
    {
        // Connection has been made.  Update status.
        tcp_connection::update_status(tcp_connection::status::CONNECTED);

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
        tcp_connection::update_status(tcp_connection::status::DISCONNECTED);
    }
}
