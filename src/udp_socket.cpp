#include "udp_socket.hpp"

#include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>

// CONSTRUCTORS
udp_socket_t::udp_socket_t(boost::asio::io_service& io_service, uint32_t id)
    : m_socket(io_service),
      m_id(id)
{

}
udp_socket_t::~udp_socket_t()
{
    udp_socket_t::close();
}

// METHODS: CONTROL
bool udp_socket_t::open(boost::asio::ip::udp::endpoint& local_endpoint)
{
    // Create error code for tracking.
    boost::system::error_code error;

    // Bind the socket.
    udp_socket_t::m_socket.bind(local_endpoint, error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to bind udp socket " << udp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Open the socket.
    udp_socket_t::m_socket.open(boost::asio::ip::udp::v4(), error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to open udp socket " << udp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Create TX/RX subscribers/publishers.
    ros::NodeHandle private_node("~");
    std::string topic_base = "sockets/" + std::to_string(udp_socket_t::m_id);
    udp_socket_t::m_subscriber_tx = private_node.subscribe(topic_base + "/tx", 100, &udp_socket_t::subscriber_tx, this);
    udp_socket_t::m_publisher_rx = private_node.advertise<driver_modem_msgs::udp_packet>(topic_base + "/rx", 100);

    // Start async receiving.
    udp_socket_t::async_rx();

    // If this point reached, socket has been opened.
    return true;
}
void udp_socket_t::close()
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
}

// METHODS: RX
void udp_socket_t::async_rx()
{
    // Start an asynchronous receive.
    udp_socket_t::m_socket.async_receive_from(boost::asio::buffer(udp_socket_t::m_buffer),
                                              udp_socket_t::m_remote_endpoint,
                                              boost::bind(&udp_socket_t::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

// SUBSCRIBERS
void udp_socket_t::subscriber_tx(const driver_modem_msgs::udp_packetConstPtr& message)
{
    // Parse remote endpoint.
    boost::asio::ip::udp::endpoint remote_endpoint;
    boost::asio::ip::address_v4::bytes_type ip_bytes;
    std::memcpy(ip_bytes.data(), message->remote_endpoint.ip.data(), 4);
    remote_endpoint.address(boost::asio::ip::address_v4(ip_bytes));
    remote_endpoint.port(message->remote_endpoint.port);

    // Send the data.
    udp_socket_t::m_socket.send_to(boost::asio::buffer(message->data), remote_endpoint);
}

// CALLBACKS
void udp_socket_t::rx_callback(const boost::system::error_code& error, std::size_t bytes_read)
{
    // Check for recieve errors.
    if(!error)
    {
        // Publish the message.
        driver_modem_msgs::udp_packet message;
        std::memcpy(message.remote_endpoint.ip.data(), udp_socket_t::m_remote_endpoint.address().to_v4().to_bytes().data(), 4);
        message.remote_endpoint.port = udp_socket_t::m_remote_endpoint.port();
        message.data.assign(udp_socket_t::m_buffer.begin(), udp_socket_t::m_buffer.begin() + bytes_read);
        udp_socket_t::m_publisher_rx.publish(message);

        // Start a new asynchronous receive.
        udp_socket_t::async_rx();
    }
    else
    {
        // Check if port is being closed.
        if(error != boost::asio::error::operation_aborted)
        {
            ROS_ERROR_STREAM("udp socket " << udp_socket_t::m_id << " asynchrounous receive failed (" << error.message() << ")");
        }
    }
}