#include "driver_modem.hpp"

#include <driver_modem_msgs/status.h>

using namespace driver_modem;

// CONSTRUCTORS
driver_modem_t::driver_modem_t()
{
    // Get private handle.
    ros::NodeHandle private_node("~");

    // Set up status publisher.
    driver_modem_t::m_publisher_status = private_node.advertise<driver_modem_msgs::status>("status", 1, true);

    // Publish the initial status.
    driver_modem_t::publish_status();

    // Start services.
    driver_modem_t::m_service_start_tcp_server = private_node.advertiseService("start_tcp_server", &driver_modem_t::service_start_tcp_server, this);
    driver_modem_t::m_service_stop_tcp_server = private_node.advertiseService("stop_tcp_server", &driver_modem_t::service_stop_tcp_server, this);
    driver_modem_t::m_service_open_tcp_socket = private_node.advertiseService("open_tcp_socket", &driver_modem_t::service_open_tcp_socket, this);
    driver_modem_t::m_service_open_udp_socket = private_node.advertiseService("open_udp_socket", &driver_modem_t::service_open_udp_socket, this);
    driver_modem_t::m_service_close_socket = private_node.advertiseService("close_socket", &driver_modem_t::service_close_socket, this);
}
driver_modem_t::~driver_modem_t()
{
    // Clean up maps.
    for(auto server = driver_modem_t::m_tcp_servers.begin(); server != driver_modem_t::m_tcp_servers.end(); ++server)
    {
        // NOTE: Deleting the instance also stops the server.
        delete server->second;
    }
    for(auto socket = driver_modem_t::m_sockets.begin(); socket != driver_modem_t::m_sockets.end(); ++socket)
    {
        // NOTE: Deleting the instance also closes the socket.
        delete socket->second;
    }
}

// CONTROL
void driver_modem_t::run()
{

}

// SERVICE CALLBACKS
bool driver_modem_t::service_start_tcp_server(driver_modem_msgs::start_tcp_serverRequest& request, driver_modem_msgs::start_tcp_serverResponse& response)
{

}
bool driver_modem_t::service_stop_tcp_server(driver_modem_msgs::stop_tcp_serverRequest& request, driver_modem_msgs::stop_tcp_serverResponse& response)
{

}
bool driver_modem_t::service_open_tcp_socket(driver_modem_msgs::open_tcp_socketRequest& request, driver_modem_msgs::open_tcp_socketResponse& response)
{

}
bool driver_modem_t::service_open_udp_socket(driver_modem_msgs::open_udp_socketRequest& request, driver_modem_msgs::open_udp_socketResponse& response)
{

}
bool driver_modem_t::service_close_socket(driver_modem_msgs::close_socketRequest& request, driver_modem_msgs::close_socketResponse& response)
{

}

// PUBLISHING
void driver_modem_t::publish_status() const
{

}