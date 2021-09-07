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
#include <driver_modem_msgs/status.h>

#include <map>

namespace driver_modem {

class driver_modem_t
{
public:
    driver_modem_t();
    ~driver_modem_t();

private:
    ros::NodeHandle m_node;
    ros::ServiceServer m_service_start_tcp_server;
    bool service_start_tcp_server(driver_modem_msgs::start_tcp_serverRequest& request, driver_modem_msgs::start_tcp_serverResponse& response);
    ros::ServiceServer m_service_stop_tcp_server;
    bool service_stop_tcp_server(driver_modem_msgs::stop_tcp_serverRequest& request, driver_modem_msgs::stop_tcp_serverResponse& response);
    ros::ServiceServer m_service_open_tcp_socket;
    bool service_open_tcp_socket(driver_modem_msgs::open_tcp_socketRequest& request, driver_modem_msgs::open_tcp_socketResponse& response);

    std::map<uint32_t, tcp_server_t*> m_tcp_servers;
    std::map<uint32_t, socket_t*> m_sockets;


};

}

#endif