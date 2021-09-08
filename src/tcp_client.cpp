#include "tcp_client.hpp"

#include "endpoint.hpp"

#include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>

using namespace driver_tcpip;

// CONSTRUCTORS
tcp_client_t::tcp_client_t(boost::asio::io_service& io_service, uint32_t id, std::function<void(boost::asio::ip::tcp::socket*)> connection_callback)
    : m_id(id)
{
    // Initialize socket.
    tcp_client_t::m_socket = new boost::asio::ip::tcp::socket(io_service);

    // Store connection callback.
    tcp_client_t::m_callback_connection = connection_callback;
}
tcp_client_t::~tcp_client_t()
{
    // Stop the connection operation.
    tcp_client_t::stop();

    // Clean up the socket.
    delete tcp_client_t::m_socket;
}

// CONTROL
bool tcp_client_t::start(const driver_tcpip_msgs::endpoint& local_endpoint, const driver_tcpip_msgs::endpoint& remote_endpoint)
{
    // Check if the client has alread failed.
    if(!tcp_client_t::m_socket)
    {
        ROS_ERROR_STREAM("tcp client " << tcp_client_t::m_id << " failed to start (failed on prior connection attempt)");
        return false;
    }

    // Check if the socket is already open.
    if(tcp_client_t::m_socket->is_open())
    {
        ROS_ERROR_STREAM("tcp client " << tcp_client_t::m_id << " failed to start (already started)");
        return false;
    }

    // Store the endpoints.
    tcp_client_t::m_local_endpoint = local_endpoint;
    tcp_client_t::m_remote_endpoint = remote_endpoint;

    // Create error structure for tracking.
    boost::system::error_code error;

    // Open the socket.
    tcp_client_t::m_socket->open(boost::asio::ip::tcp::v4());

    // Bind the socket to the local endpoint.
    tcp_client_t::m_socket->bind(endpoint::to_asio_tcp(local_endpoint), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to bind tcp client " << tcp_client_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Start an asynchronous connect operation.
    tcp_client_t::m_socket->async_connect(endpoint::to_asio_tcp(remote_endpoint), boost::bind(&tcp_client_t::connect_callback, this, boost::asio::placeholders::error));

    // Indicate connection started.
    ROS_INFO_STREAM("tcp client " << tcp_client_t::m_id << " started successfully");
    
    return true;
}
void tcp_client_t::stop()
{
    // Close the socket.
    if(tcp_client_t::m_socket)
    {
        // Close the socket if open.
        if(tcp_client_t::m_socket->is_open())
        {
            boost::system::error_code error;
            tcp_client_t::m_socket->close(error);
            if(!error)
            {
                ROS_INFO_STREAM("tcp client " << tcp_client_t::m_id << " stopped successfully");
            }
            else
            {
                ROS_ERROR_STREAM("tcp client " << tcp_client_t::m_id << " failed to stop (" << error.message() << ")");
            }
        }
        
        // Clean up the socket instance.
        delete tcp_client_t::m_socket;
        tcp_client_t::m_socket = nullptr;
    }

}

// PROPERTIES
driver_tcpip_msgs::tcp_client tcp_client_t::description() const
{
    // Create output description.
    driver_tcpip_msgs::tcp_client description;

    // Populate description.
    description.id = tcp_client_t::m_id;
    description.local_endpoint = tcp_client_t::m_local_endpoint;
    description.remote_endpoint = tcp_client_t::m_remote_endpoint;

    // Return description.
    return description;
}
bool tcp_client_t::is_active() const
{
    return tcp_client_t::m_socket != nullptr;
}

// ASIO SOCKET
void tcp_client_t::connect_callback(const boost::system::error_code &error)
{
    // Check if an error occured.
    if(error)
    {
        // Log failure.
        ROS_ERROR_STREAM("tcp client " << tcp_client_t::m_id << " failed to connect (" << error.message() << ")");

        // Stop the client.
        tcp_client_t::stop();

        return;
    }

    // Indicate success.
    ROS_INFO_STREAM("tcp client " << tcp_client_t::m_id << " connected successfully");

    // Raise external connection callback.
    tcp_client_t::m_callback_connection(tcp_client_t::m_socket);

    // Socket pointer ownership has been passed via external callback.
    tcp_client_t::m_socket = nullptr;
}