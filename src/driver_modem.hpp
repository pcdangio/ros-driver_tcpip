/// \file driver_modem.hpp
/// \brief Defines the driver_modem::driver_modem_t class.
#ifndef DRIVER_MODEM___DRIVER_MODEM_H
#define DRIVER_MODEM___DRIVER_MODEM_H

#include "tcp_server.hpp"
#include "socket.hpp"

#include <ros/ros.h>
#include <driver_modem_msgs/start_tcp_server.h>
#include <driver_modem_msgs/stop_tcp_server.h>
#include <driver_modem_msgs/open_tcp_socket.h>
#include <driver_modem_msgs/open_udp_socket.h>
#include <driver_modem_msgs/close_socket.h>

#include <map>

/// \brief Includes all software for TCP/IP communication over a network.
namespace driver_modem {

/// \brief A ROS node for controlling TCP/IP sockets.
class driver_modem_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new driver_modem_t.
    driver_modem_t();
    ~driver_modem_t();

    // CONTROL
    /// \brief Runs the node until shutdown.
    void run();

private:
    // NODE
    /// \brief The node's handle.
    ros::NodeHandle m_node;

    // ASIO SERVICE
    /// \brief The application's io_service instance.
    boost::asio::io_service m_io_service;

    // MAPS
    /// \brief The collection of active TCP servers.
    std::map<uint32_t, tcp_server_t*> m_tcp_servers;
    /// \brief The collection of open TCP or UDP sockets.
    std::map<uint32_t, socket_t*> m_sockets;

    // PUBLISHERS
    /// \brief The publisher for status messages.
    ros::Publisher m_publisher_status;

    // SERVICE SERVERS
    /// \brief The service for starting a TCP server.
    ros::ServiceServer m_service_start_tcp_server;
    /// \brief The service for stopping a TCP server. 
    ros::ServiceServer m_service_stop_tcp_server;
    /// \brief The service for opening a TCP socket as a client.
    ros::ServiceServer m_service_open_tcp_socket;
    /// \brief The service for opening a UDP socket.
    ros::ServiceServer m_service_open_udp_socket;
    /// \brief The service for closing a TCP or UDP socket.
    ros::ServiceServer m_service_close_socket;

    // SERVICE CALLBACKS
    /// \brief The service callback for starting a TCP server.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service executed successfully, otherwise FALSE.
    bool service_start_tcp_server(driver_modem_msgs::start_tcp_serverRequest& request, driver_modem_msgs::start_tcp_serverResponse& response);
    /// \brief The service callback for stopping a TCP server.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service executed successfully, otherwise FALSE.
    bool service_stop_tcp_server(driver_modem_msgs::stop_tcp_serverRequest& request, driver_modem_msgs::stop_tcp_serverResponse& response);
    /// \brief The service callback for opening a TCP socket as a client.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service executed successfully, otherwise FALSE.
    bool service_open_tcp_socket(driver_modem_msgs::open_tcp_socketRequest& request, driver_modem_msgs::open_tcp_socketResponse& response);
    /// \brief The service callback for opening a UDP socket.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service executed successfully, otherwise FALSE.
    bool service_open_udp_socket(driver_modem_msgs::open_udp_socketRequest& request, driver_modem_msgs::open_udp_socketResponse& response);
    /// \brief The service callback for closing a TCP or UDP socket.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service executed successfully, otherwise FALSE.
    bool service_close_socket(driver_modem_msgs::close_socketRequest& request, driver_modem_msgs::close_socketResponse& response);

    // PUBLISHING
    /// \brief Publishes the current status of the modem.
    void publish_status() const;

    // CONNECTION
    /// \brief Handles new connections from TCP servers.
    /// \param socket The ASIO socket that was opened as a result of the connection.
    void tcp_connection(uint32_t id, boost::asio::ip::tcp::socket* socket);
};

}

#endif