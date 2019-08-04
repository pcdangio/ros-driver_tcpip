#include "driver_modem/modem_interface.h"

#include <driver_modem/SetRemoteHost.h>
#include <driver_modem/GetRemoteHost.h>
#include <driver_modem/AddTCPConnection.h>
#include <driver_modem/AddUDPConnection.h>
#include <driver_modem/RemoveConnection.h>

// CONSTRUCTORS
modem_interface::modem_interface(std::string modem_namespace)
{
    // Get node handle.
    ros::NodeHandle node;

    // Initialize service clients.
    modem_interface::m_service_set_remote_host = node.serviceClient<driver_modem::SetRemoteHost>(modem_namespace + "/set_remote_host");
    modem_interface::m_service_get_remote_host = node.serviceClient<driver_modem::GetRemoteHost>(modem_namespace + "/get_remote_host");
    modem_interface::m_service_add_tcp_connection = node.serviceClient<driver_modem::AddTCPConnection>(modem_namespace + "/add_tcp_connection");
    modem_interface::m_service_add_udp_connection = node.serviceClient<driver_modem::AddUDPConnection>(modem_namespace + "/add_udp_connection");
    modem_interface::m_service_remove_connection = node.serviceClient<driver_modem::RemoveConnection>(modem_namespace + "/remove_connection");
}
modem_interface::~modem_interface()
{
    // Shut down service clients.
    modem_interface::m_service_set_remote_host.shutdown();
    modem_interface::m_service_get_remote_host.shutdown();
    modem_interface::m_service_add_tcp_connection.shutdown();
    modem_interface::m_service_add_udp_connection.shutdown();
    modem_interface::m_service_remove_connection.shutdown();
}

// METHODS: Connection Management
bool modem_interface::set_remote_host(std::string remote_host)
{
    // Build request.
    driver_modem::SetRemoteHost service;
    service.request.remote_host = remote_host;

    // Call service.
    if(modem_interface::m_service_set_remote_host.call(service))
    {
        return service.response.success;
    }
    else
    {
        return false;
    }
}
bool modem_interface::get_remote_host(std::string& remote_host)
{
    // Build request.
    driver_modem::GetRemoteHost service;

    // Call service.
    if(modem_interface::m_service_get_remote_host.call(service))
    {
        remote_host = service.response.remote_host;
        return true;
    }
    else
    {
        return false;
    }
}
bool modem_interface::add_tcp_connection(modem_interface::tcp_role role, uint16_t port)
{
    // Build request.
    driver_modem::AddTCPConnection service;
    service.request.role = static_cast<uint8_t>(role);
    service.request.port = port;

    // Call service.
    if(modem_interface::m_service_add_tcp_connection.call(service))
    {
        return service.response.success;
    }
    else
    {
        return false;
    }
}
bool modem_interface::add_udp_connection(uint16_t port)
{
    // Build request.
    driver_modem::AddUDPConnection service;
    service.request.port = port;

    // Call service.
    if(modem_interface::m_service_add_udp_connection.call(service))
    {
        return service.response.success;
    }
    else
    {
        return false;
    }
}
bool modem_interface::remove_connection(protocol type, uint16_t port)
{
    // Build request.
    driver_modem::RemoveConnection service;
    service.request.protocol = static_cast<uint8_t>(type);
    service.request.port = port;

    // Call service.
    if(modem_interface::m_service_remove_connection.call(service))
    {
        return service.response.success;
    }
    else
    {
        return false;
    }
}

// METHODS: Data Transmission
bool modem_interface::send_tcp(uint8_t port, const uint8_t *data, uint32_t length)
{

}
bool modem_interface::send_udp(uint8_t port, const uint8_t *data, uint32_t length)
{

}
