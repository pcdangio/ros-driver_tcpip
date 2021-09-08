#include "endpoint.hpp"

// TCP
boost::asio::ip::tcp::endpoint driver_modem::endpoint::to_asio_tcp(const driver_modem_msgs::endpoint& endpoint_ros)
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
driver_modem_msgs::endpoint driver_modem::endpoint::to_ros(const boost::asio::ip::tcp::endpoint& endpoint_asio)
{
    // Create ROS endpoint output.
    driver_modem_msgs::endpoint endpoint_ros;

    // Set endpoint address.
    std::memcpy(endpoint_ros.ip.data(), endpoint_asio.address().to_v4().to_bytes().data(), 4);

    // Set endpoint port.
    endpoint_ros.port = endpoint_asio.port();

    return endpoint_ros;
}

// UDP
boost::asio::ip::udp::endpoint driver_modem::endpoint::to_asio_udp(const driver_modem_msgs::endpoint& endpoint_ros)
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
driver_modem_msgs::endpoint driver_modem::endpoint::to_ros(const boost::asio::ip::udp::endpoint& endpoint_asio)
{
    // Create ROS endpoint output.
    driver_modem_msgs::endpoint endpoint_ros;

    // Set endpoint address.
    std::memcpy(endpoint_ros.ip.data(), endpoint_asio.address().to_v4().to_bytes().data(), 4);

    // Set endpoint port.
    endpoint_ros.port = endpoint_asio.port();

    return endpoint_ros;
}