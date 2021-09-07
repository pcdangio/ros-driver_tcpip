#include "tcp_socket.hpp"

#include <driver_modem_msgs/tcp_packet.h>

#include <boost/asio/placeholders.hpp>

using namespace driver_modem;

// CONSTRUCTORS
tcp_socket_t::tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id)
    : socket_t(id, protocol_t::TCP)
{
    // Store socket.
    tcp_socket_t::m_socket = socket;

    // Start ROS publishers and services.
    tcp_socket_t::start_ros();
}
tcp_socket_t::tcp_socket_t(boost::asio::io_service& io_service, uint32_t id)
    : socket_t(id, protocol_t::TCP)
{
    // Create new socket.
    tcp_socket_t::m_socket = new boost::asio::ip::tcp::socket(io_service);
}
tcp_socket_t::~tcp_socket_t()
{
    // Close the socket.
    tcp_socket_t::close();

    // Free the socket.
    delete tcp_socket_t::m_socket;
}

// CONTROL
bool tcp_socket_t::connect(driver_modem_msgs::endpoint& local_endpoint, driver_modem_msgs::endpoint& remote_endpoint)
{
    // Check if the socket is already open.
    if(tcp_socket_t::m_socket->is_open())
    {
        ROS_ERROR_STREAM("tcp socket " << tcp_socket_t::m_id << " failed to connect (socket is already connected)");
        return false;
    }

    // Create error structure for tracking.
    boost::system::error_code error;

    // Bind the socket to the local endpoint.
    tcp_socket_t::m_socket->bind(tcp_socket_t::endpoint_asio(local_endpoint), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to bind tcp socket " << tcp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Open socket and connect to remote endpoint.
    tcp_socket_t::m_socket->connect(tcp_socket_t::endpoint_asio(remote_endpoint), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to connect tcp socket " << tcp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Start ROS publishers and services.
    tcp_socket_t::start_ros();

    // Start asynchronously receiving.
    tcp_socket_t::async_rx();

    // Indicate success.
    ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " connected successfully");
    return false;
}
void tcp_socket_t::close()
{
    if(tcp_socket_t::m_socket->is_open())
    {
        // Stop publishers and services.
        tcp_socket_t::stop_ros();

        // Close the ASIO socket.
        boost::system::error_code error;
        tcp_socket_t::m_socket->close(error);
        if(error)
        {
            ROS_ERROR_STREAM("failed to close udp socket " << tcp_socket_t::m_id << " (" << error.message() << ")");
        }
    }
}

// PROPERTIES
driver_modem_msgs::tcp_socket tcp_socket_t::description() const
{
    // Create output message.
    driver_modem_msgs::tcp_socket description;

    // Populate message.
    description.id = tcp_socket_t::m_id;
    description.local_endpoint = tcp_socket_t::endpoint_ros(tcp_socket_t::m_socket->local_endpoint());
    description.remote_endpoint = tcp_socket_t::endpoint_ros(tcp_socket_t::m_socket->remote_endpoint());

    return description;
}
bool tcp_socket_t::is_open() const
{
    return tcp_socket_t::is_open();
}

// ASIO SOCKET
void tcp_socket_t::async_rx()
{
    // Start an asynchronous receive.
    tcp_socket_t::m_socket->async_receive(boost::asio::buffer(tcp_socket_t::m_buffer),
                                          boost::bind(&tcp_socket_t::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}
void tcp_socket_t::rx_callback(const boost::system::error_code& error, std::size_t bytes_read)
{
    // Check for recieve errors.
    if(!error)
    {
        // Publish the message.
        driver_modem_msgs::tcp_packet message;
        message.data.assign(tcp_socket_t::m_buffer.begin(), tcp_socket_t::m_buffer.begin() + bytes_read);
        tcp_socket_t::m_publisher_rx.publish(message);
    }
    else
    {
        // An error occured.

        // Check if port is being closed and rx was aborted.
        if(error == boost::asio::error::operation_aborted)
        {
            // Quit receiving.
            return;
        }

        // Check if remote closed the connection.
        if(error == boost::asio::error::connection_reset)
        {
            // Close socket.
            tcp_socket_t::close();

            // Quit receiving.
            return;
        }

        // Otherwise, report error.
        ROS_ERROR_STREAM("tcp socket " << tcp_socket_t::m_id << " asynchrounous receive failed (" << error.message() << ")");
    }

    // Continue receiving data.
    tcp_socket_t::async_rx();
}


// ROS
void tcp_socket_t::start_ros()
{
    // Get private node handle.
    ros::NodeHandle private_node("~");
    // Create base topic.
    std::string topic_base = "sockets/" + std::to_string(tcp_socket_t::m_id);
    // Create TX service.
    tcp_socket_t::m_service_tx = private_node.advertiseService(topic_base + "/tx", &tcp_socket_t::service_tx, this);
    // Create RX publisher.
    tcp_socket_t::m_publisher_rx = private_node.advertise<driver_modem_msgs::tcp_packet>(topic_base + "/rx", 100);
}
void tcp_socket_t::stop_ros()
{
    tcp_socket_t::m_service_tx.shutdown();
    tcp_socket_t::m_publisher_rx.shutdown();
}
bool tcp_socket_t::service_tx(driver_modem_msgs::send_tcpRequest& request, driver_modem_msgs::send_tcpResponse& response)
{
    // Try sending the message.
    boost::system::error_code error;
    tcp_socket_t::m_socket->send(boost::asio::buffer(request.packet.data), 0, error);

    // Indicate if message was sent successfully.
    response.success = !error;

    // Indicate that service was executed successfully.
    return true;
}

// ENDPOINT CONVERSION
boost::asio::ip::tcp::endpoint tcp_socket_t::endpoint_asio(const driver_modem_msgs::endpoint& endpoint_ros) const
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
driver_modem_msgs::endpoint tcp_socket_t::endpoint_ros(const boost::asio::ip::tcp::endpoint& endpoint_asio) const
{
    // Create ROS endpoint output.
    driver_modem_msgs::endpoint endpoint_ros;

    // Set endpoint address.
    std::memcpy(endpoint_ros.ip.data(), endpoint_asio.address().to_v4().to_bytes().data(), 4);

    // Set endpoint port.
    endpoint_ros.port = endpoint_asio.port();

    return endpoint_ros;
}