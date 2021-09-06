#include "tcp_socket.hpp"

#include <driver_modem_msgs/tcp_packet.h>

using namespace driver_modem;

// CONSTRUCTORS
tcp_socket_t::tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id)
    : m_id(id)
{
    // Store socket.
    tcp_socket_t::m_socket = socket;

    // Start ROS publishers and services.
    tcp_socket_t::start_ros();
}
tcp_socket_t::tcp_socket_t(boost::asio::io_service& io_service, uint32_t id)
    : m_id(id)
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
    boost::asio::ip::tcp::endpoint local_endpoint_asio = tcp_socket_t::endpoint_asio(local_endpoint);
    tcp_socket_t::m_socket->bind(local_endpoint_asio, error);
    if(error)
    {
        ROS_ERROR_STREAM("failed to bind tcp socket " << tcp_socket_t::m_id << " (" << error.message() << ")");
        return false;
    }

    // Open socket and connect to remote endpoint.
    boost::asio::ip::tcp::endpoint remote_endpoint_asio = tcp_socket_t::endpoint_asio(remote_endpoint);
    tcp_socket_t::m_socket->connect(remote_endpoint_asio, error);
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