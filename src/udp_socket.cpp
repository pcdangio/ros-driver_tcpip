#include "udp_socket.hpp"

#include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>

using namespace driver_modem;

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

// CONTROL
bool udp_socket_t::open(driver_modem_msgs::endpoint& local_endpoint)
{
    // Check if socket is already open.
    if(udp_socket_t::m_socket.is_open())
    {
        ROS_ERROR_STREAM("failed to open udp socket " << udp_socket_t::m_id << " (socket is already open)");
        return false;
    }
    
    // Create error code for tracking.
    boost::system::error_code error;

    // Bind the socket.
    boost::asio::ip::udp::endpoint local_endpoint_asio = udp_socket_t::endpoint_asio(local_endpoint);
    udp_socket_t::m_socket.bind(local_endpoint_asio, error);
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

    // Set publishers and subscribers.
    // Get private node handle.
    ros::NodeHandle private_node("~");
    // Set base topic name.
    std::string topic_base = "sockets/" + std::to_string(udp_socket_t::m_id);
    // Create TX subscriber.
    udp_socket_t::m_subscriber_tx = private_node.subscribe(topic_base + "/tx", 100, &udp_socket_t::subscriber_tx, this);
    // Create RX publisher.
    udp_socket_t::m_publisher_rx = private_node.advertise<driver_modem_msgs::udp_packet>(topic_base + "/rx", 100);

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
    }
}

// PROPERTIES
driver_modem_msgs::udp_socket udp_socket_t::description() const
{
    // Create output message.
    driver_modem_msgs::udp_socket description;

    // Populate output message.
    description.id = udp_socket_t::m_id;
    description.local_endpoint = udp_socket_t::endpoint_ros(udp_socket_t::m_socket.local_endpoint());

    return description;
}

// SUBSCRIBERS
void udp_socket_t::subscriber_tx(const driver_modem_msgs::udp_packetConstPtr& message)
{
    // Send the data.
    udp_socket_t::m_socket.send_to(boost::asio::buffer(message->data), udp_socket_t::endpoint_asio(message->remote_endpoint));
}

// RX
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
        driver_modem_msgs::udp_packet message;
        message.remote_endpoint = udp_socket_t::endpoint_ros(udp_socket_t::m_remote_endpoint);
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

// ENDPOINT CONVERSION
boost::asio::ip::udp::endpoint udp_socket_t::endpoint_asio(const driver_modem_msgs::endpoint& endpoint_ros) const
{
    // Create ASIO endpoint output.
    boost::asio::ip::udp::endpoint endpoint_asio;

    // Set the endpoint address.
    boost::asio::ip::address_v4::bytes_type ip_bytes;
    std::memcpy(ip_bytes.data(), endpoint_ros.ip.data(), 4);
    endpoint_asio.address(boost::asio::ip::address_v4(ip_bytes));

    // Set the endpoint port.
    endpoint_asio.port(endpoint_ros.port);

    return endpoint_asio;
}
driver_modem_msgs::endpoint udp_socket_t::endpoint_ros(const boost::asio::ip::udp::endpoint& endpoint_asio) const
{
    // Create ROS endpoint output.
    driver_modem_msgs::endpoint endpoint_ros;

    // Set endpoint address.
    std::memcpy(endpoint_ros.ip.data(), endpoint_asio.address().to_v4().to_bytes().data(), 4);

    // Set endpoint port.
    endpoint_ros.port = endpoint_asio.port();

    return endpoint_ros;
}