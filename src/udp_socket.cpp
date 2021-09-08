#include "udp_socket.hpp"

#include "endpoint.hpp"

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>

using namespace driver_tcpip;

// CONSTRUCTORS
udp_socket_t::udp_socket_t(boost::asio::io_service& io_service, uint32_t id)
    : socket_t(id, protocol_t::UDP),
      m_socket(io_service)
{

}
udp_socket_t::~udp_socket_t()
{
    udp_socket_t::close();
}

// CONTROL
bool udp_socket_t::open(driver_tcpip_msgs::endpoint& local_endpoint)
{
    // Check if socket is already open.
    if(udp_socket_t::m_socket.is_open())
    {
        ROS_ERROR_STREAM("failed to open udp socket " << udp_socket_t::m_id << " (socket is already open)");
        return false;
    }
    
    // Create error code for tracking.
    boost::system::error_code error;

    // Open the socket.
    udp_socket_t::m_socket.open(boost::asio::ip::udp::v4(), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to open udp socket " << udp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Bind the socket.
    udp_socket_t::m_socket.bind(endpoint::to_asio_udp(local_endpoint), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to bind udp socket " << udp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Set publishers and subscribers.
    // Get private node handle.
    ros::NodeHandle private_node("~");
    // Set base topic name.
    std::string topic_base = "sockets/" + std::to_string(udp_socket_t::m_id);
    // Create TX subscriber.
    udp_socket_t::m_subscriber_tx = private_node.subscribe(topic_base + "/tx", 100, &udp_socket_t::subscriber_tx, this);
    // Create RX publisher.
    udp_socket_t::m_publisher_rx = private_node.advertise<driver_tcpip_msgs::udp_packet>(topic_base + "/rx", 100);

    // Start async receiving.
    udp_socket_t::async_rx();

    // Indicate that socket has been opened successfully.
    ROS_INFO_STREAM("udp socket " << udp_socket_t::m_id << " opened successfully");
    return true;
}
void udp_socket_t::close()
{
    if(udp_socket_t::m_socket.is_open())
    {
        // Close TX/RX subscribers/publishers
        udp_socket_t::m_subscriber_tx.shutdown();
        udp_socket_t::m_publisher_rx.shutdown();

        // Close the socket.
        boost::system::error_code error;
        udp_socket_t::m_socket.close(error);
        if(error)
        {
            ROS_ERROR_STREAM("failed to close udp socket " << udp_socket_t::m_id << " (" << error.message() << ")");
        }
        else
        {
            ROS_INFO_STREAM("udp socket " << udp_socket_t::m_id << " closed successfully");
        }
    }
}

// PROPERTIES
driver_tcpip_msgs::udp_socket udp_socket_t::description() const
{
    // Create output message.
    driver_tcpip_msgs::udp_socket description;

    // Populate output message.
    description.id = udp_socket_t::m_id;
    description.local_endpoint = endpoint::to_ros(udp_socket_t::m_socket.local_endpoint());

    return description;
}
bool udp_socket_t::is_open() const
{
    return udp_socket_t::m_socket.is_open();
}

// ASIO SOCKET
void udp_socket_t::async_rx()
{
    // Start an asynchronous receive.
    udp_socket_t::m_socket.async_receive_from(boost::asio::buffer(udp_socket_t::m_buffer),
                                              udp_socket_t::m_remote_endpoint,
                                              boost::bind(&udp_socket_t::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}
void udp_socket_t::rx_callback(const boost::system::error_code& error, std::size_t bytes_read)
{
    // Check for recieve errors.
    if(!error)
    {
        // Publish the message.
        driver_tcpip_msgs::udp_packet message;
        message.remote_endpoint = endpoint::to_ros(udp_socket_t::m_remote_endpoint);
        message.data.assign(udp_socket_t::m_buffer.begin(), udp_socket_t::m_buffer.begin() + bytes_read);
        udp_socket_t::m_publisher_rx.publish(message);
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

        // Otherwise, report error.
        ROS_ERROR_STREAM("udp socket " << udp_socket_t::m_id << " asynchrounous receive failed (" << error.message() << ")");
    }

    // Continue receiving data.
    udp_socket_t::async_rx();
}

// ROS
void udp_socket_t::subscriber_tx(const driver_tcpip_msgs::udp_packetConstPtr& message)
{
    // Send the data.
    udp_socket_t::m_socket.send_to(boost::asio::buffer(message->data), endpoint::to_asio_udp(message->remote_endpoint));
}