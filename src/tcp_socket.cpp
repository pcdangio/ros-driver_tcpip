#include "tcp_socket.hpp"

#include "endpoint.hpp"

#include <driver_tcpip_msgs/tcp_packet.h>

#include <boost/asio/placeholders.hpp>

using namespace driver_tcpip;

// CONSTRUCTORS
tcp_socket_t::tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id)
    : socket_t(id, protocol_t::TCP)
{
    // Store socket.
    tcp_socket_t::m_socket = socket;

    // Start ROS publishers and services.
    tcp_socket_t::start_ros();

    // Start asynchronously receiving.
    tcp_socket_t::async_rx();

    // Log socket opening.
    ROS_INFO_STREAM("tcp socket " << id << " opened successfully");
}
tcp_socket_t::~tcp_socket_t()
{
    // Close the socket.
    tcp_socket_t::close();

    // Free the socket.
    delete tcp_socket_t::m_socket;
}

// CONTROL
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
        else
        {
            ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " closed successfully");
        }
    }
}

// PROPERTIES
driver_tcpip_msgs::tcp_socket tcp_socket_t::description() const
{
    // Create output message.
    driver_tcpip_msgs::tcp_socket description;

    // Populate message.
    description.id = tcp_socket_t::m_id;
    description.local_endpoint = endpoint::to_ros(tcp_socket_t::m_socket->local_endpoint());
    description.remote_endpoint = endpoint::to_ros(tcp_socket_t::m_socket->remote_endpoint());

    return description;
}
bool tcp_socket_t::is_open() const
{
    return tcp_socket_t::m_socket->is_open();
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
        driver_tcpip_msgs::tcp_packet message;
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
        if(error == boost::asio::error::connection_reset || error == boost::asio::error::eof)
        {
            // Indicate remote has closed.
            ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " disconnect by peer");
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
    tcp_socket_t::m_publisher_rx = private_node.advertise<driver_tcpip_msgs::tcp_packet>(topic_base + "/rx", 100);
}
void tcp_socket_t::stop_ros()
{
    tcp_socket_t::m_service_tx.shutdown();
    tcp_socket_t::m_publisher_rx.shutdown();
}
bool tcp_socket_t::service_tx(driver_tcpip_msgs::send_tcpRequest& request, driver_tcpip_msgs::send_tcpResponse& response)
{
    // Try sending the message.
    boost::system::error_code error;
    tcp_socket_t::m_socket->send(boost::asio::buffer(request.packet.data), 0, error);

    // Indicate if the message was sent successfully.
    return !error;
}