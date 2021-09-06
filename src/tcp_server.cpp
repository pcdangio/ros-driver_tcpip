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
    // Check if server is already running.
    if(tcp_server_t::m_acceptor.is_open())
    {
        ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to start (server is already running)");
        return false;
    }

    // Create error structure for tracking.
    boost::system::error_code error;

    // Bind the acceptor.
    tcp_server_t::m_acceptor.set_option(boost::asio::socket_base::reuse_address(true));
    tcp_server_t::m_acceptor.bind(tcp_server_t::endpoint_asio(local_endpoint), error);
    if(error)
    {
        ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to bind to local endpoint (" << error.message() << ")");
        return false;
    }

    // Open the acceptor.
    tcp_server_t::m_acceptor.open(tcp_server_t::m_acceptor.local_endpoint().protocol(), error);
    if(error)
    {
        ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to open (" << error.message() << ")");
        return false;
    }

    // Instruct the acceptor to listen for connections.
    tcp_server_t::m_acceptor.listen(128, error);
    if(error)
    {
        ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to enter listening mode (" << error.message() << ")");
        return false;
    }

    // If this point reached, acceptor is open.

    // Start asynchronously accepting connections.
    tcp_server_t::async_accept();

    // Indicate success.
    ROS_INFO_STREAM("tcp server " << tcp_server_t::m_id << " successfully started");
    return true;
}
void tcp_server_t::stop()
{
    // Check if acceptor is running.
    if(tcp_server_t::m_acceptor.is_open())
    {
        // Stop the acceptor.
        boost::system::error_code error;
        tcp_server_t::m_acceptor.close(error);
        if(error)
        {
            ROS_ERROR_STREAM("tcp server " << tcp_server_t::m_id << " failed to halt (" << error.message() << ")");
        }
    }
}

// PROPERTIES
driver_modem_msgs::tcp_server tcp_server_t::description() const
{
    // Create output description.
    driver_modem_msgs::tcp_server description;

    // Populate description.
    description.id = tcp_server_t::m_id;
    description.local_endpoint = tcp_server_t::endpoint_ros(tcp_server_t::m_acceptor.local_endpoint());

    return description;
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

// ENDPOINT CONVERSION
boost::asio::ip::tcp::endpoint tcp_server_t::endpoint_asio(const driver_modem_msgs::endpoint& endpoint_ros) const
{
    // Create ASIO endpoint output.
    boost::asio::ip::tcp::endpoint endpoint_asio;

    // Set the endpoint address.
    boost::asio::ip::address_v4::bytes_type ip_bytes;
    std::memcpy(ip_bytes.data(), endpoint_ros.ip.data(), 4);
    endpoint_asio.address(boost::asio::ip::address_v4(ip_bytes));

    // Set the endpoint port.
    endpoint_asio.port(endpoint_ros.port);

    return endpoint_asio;
}
driver_modem_msgs::endpoint tcp_server_t::endpoint_ros(const boost::asio::ip::tcp::endpoint& endpoint_asio) const
{
    // Create ROS endpoint output.
    driver_modem_msgs::endpoint endpoint_ros;

    // Set endpoint address.
    std::memcpy(endpoint_ros.ip.data(), endpoint_asio.address().to_v4().to_bytes().data(), 4);

    // Set endpoint port.
    endpoint_ros.port = endpoint_asio.port();

    return endpoint_ros;
}