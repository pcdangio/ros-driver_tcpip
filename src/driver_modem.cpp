#include "driver_modem.hpp"

#include <driver_modem_msgs/status.h>

using namespace driver_modem;

// CONSTRUCTORS
driver_modem_t::driver_modem_t()
{

}
driver_modem_t::~driver_modem_t()
{

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