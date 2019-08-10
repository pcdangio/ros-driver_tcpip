#include "driver_modem/modem_interface.h"

#include <driver_modem/SetRemoteHost.h>
#include <driver_modem/GetRemoteHost.h>
#include <driver_modem/AddTCPConnection.h>
#include <driver_modem/AddUDPConnection.h>
#include <driver_modem/RemoveConnection.h>
#include <driver_modem/SendTCP.h>

using namespace driver_modem;

// CONSTRUCTORS
modem_interface::modem_interface(std::string modem_name)
{
    // Get node handle.
    modem_interface::m_node = new ros::NodeHandle(modem_name);

    // Initialize active connections subscriber.
    modem_interface::m_subscriber_active_connections = modem_interface::m_node->subscribe("active_connections", 1, &modem_interface::callback_active_connections, this);

    // Initialize connection management service clients.
    modem_interface::m_service_set_remote_host = modem_interface::m_node->serviceClient<driver_modem::SetRemoteHost>("set_remote_host");
    modem_interface::m_service_get_remote_host = modem_interface::m_node->serviceClient<driver_modem::GetRemoteHost>("get_remote_host");
    modem_interface::m_service_add_tcp_connection = modem_interface::m_node->serviceClient<driver_modem::AddTCPConnection>("add_tcp_connection");
    modem_interface::m_service_add_udp_connection = modem_interface::m_node->serviceClient<driver_modem::AddUDPConnection>("add_udp_connection");
    modem_interface::m_service_remove_connection = modem_interface::m_node->serviceClient<driver_modem::RemoveConnection>("remove_connection");
}
modem_interface::~modem_interface()
{
    // Shut down active connections subscriber.
    modem_interface::m_subscriber_active_connections.shutdown();

    // Shut down service clients.
    modem_interface::m_service_set_remote_host.shutdown();
    modem_interface::m_service_get_remote_host.shutdown();
    modem_interface::m_service_add_tcp_connection.shutdown();
    modem_interface::m_service_add_udp_connection.shutdown();
    modem_interface::m_service_remove_connection.shutdown();

    // Shut down transmission publishers, subscribers, and service clients.
    for(auto it = modem_interface::m_services_send_tcp.begin(); it != modem_interface::m_services_send_tcp.end(); it++)
    {
        it->second.shutdown();
    }
    for(auto it = modem_interface::m_publishers_udp.begin(); it != modem_interface::m_publishers_udp.end(); it++)
    {
        it->second.shutdown();
    }
    for(auto it = modem_interface::m_subscribers_tcp_rx.begin(); it != modem_interface::m_subscribers_tcp_rx.end(); it++)
    {
        it->second.shutdown();
    }
    for(auto it = modem_interface::m_subscribers_udp_rx.begin(); it != modem_interface::m_subscribers_udp_rx.end(); it++)
    {
        it->second.shutdown();
    }

    // Delete the nodehandle pointer.
    delete modem_interface::m_node;
}

// METHODS: Callback Management
void modem_interface::attach_callback_tcp_rx(std::function<void (uint16_t, const DataPacketConstPtr &)> callback)
{
    modem_interface::m_callback_tcp_rx = callback;
}
void modem_interface::detach_callback_tcp_rx()
{
    modem_interface::m_callback_tcp_rx = nullptr;
}
void modem_interface::attach_callback_udp_rx(std::function<void (uint16_t, const driver_modem::DataPacketConstPtr &)> callback)
{
    modem_interface::m_callback_udp_rx = callback;
}
void modem_interface::detach_callback_udp_rx()
{
    modem_interface::m_callback_udp_rx = nullptr;
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
bool modem_interface::add_tcp_connection(tcp_role role, uint16_t port)
{
    if(modem_interface::is_connected(protocol::TCP, port) || modem_interface::is_pending(port))
    {
        return true;
    }
    else
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
}
bool modem_interface::add_udp_connection(uint16_t port)
{
    if(modem_interface::is_connected(protocol::UDP, port))
    {
        return true;
    }
    else
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
}
bool modem_interface::remove_connection(protocol type, uint16_t port)
{
    // Check if connection is active or pending.
    switch(type)
    {
    case protocol::UDP:
    {
        if(!modem_interface::is_connected(type, port))
        {
            return true;
        }
        break;
    }
    case protocol::TCP:
    {
        if(!modem_interface::is_connected(type, port) && !modem_interface::is_pending(port))
        {
            return true;
        }
        break;
    }
    }

    // If this point reached, the connection active or pending.

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
bool modem_interface::send_tcp(uint16_t port, const uint8_t *data, uint32_t length)
{
    // Check if connection exists.
    if(modem_interface::m_services_send_tcp.count(port) != 0)
    {
        // Create message.
        driver_modem::SendTCP service;
        // NOTE: Timestamp and source IP are set on receiving end.
        service.request.packet.data.reserve(length);
        for(uint32_t i = 0; i < length; i++)
        {
            service.request.packet.data.push_back(data[i]);
        }

        // Send message.
        if(modem_interface::m_services_send_tcp.at(port).call(service))
        {
            return service.response.success;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
bool modem_interface::send_udp(uint16_t port, const uint8_t *data, uint32_t length)
{
    // Check if connection exists.
    if(modem_interface::m_publishers_udp.count(port) != 0)
    {
        // Create message.
        driver_modem::DataPacket message;
        // NOTE: Timestamp and source IP are set on receiving end.
        message.data.reserve(length);
        for(uint32_t i = 0; i < length; i++)
        {
            message.data.push_back(data[i]);
        }

        // Send message.
        modem_interface::m_publishers_udp.at(port).publish(message);

        return true;
    }
    else
    {
        return false;
    }
}

// METHODS: Connection Checking
bool modem_interface::wait_for_modem(ros::Duration timeout)
{
    // Use waifforservice to wait for the RemoveConnection service.
    return modem_interface::m_service_remove_connection.waitForExistence(timeout);
}
bool modem_interface::is_connected(protocol type, uint16_t port) const
{
    switch(type)
    {
    case protocol::TCP:
    {
        for(auto it = modem_interface::m_active_tcp_connections.begin(); it != modem_interface::m_active_tcp_connections.end(); it++)
        {
            if(*it == port)
            {
                return true;
            }
        }
        return false;
    }
    case protocol::UDP:
    {
        for(auto it = modem_interface::m_active_udp_connections.begin(); it != modem_interface::m_active_udp_connections.end(); it++)
        {
            if(*it == port)
            {
                return true;
            }
        }
        return false;
    }
    }

}
bool modem_interface::is_pending(uint16_t port) const
{
    for(auto it = modem_interface::m_pending_tcp_connections.begin(); it != modem_interface::m_pending_tcp_connections.end(); it++)
    {
        if(*it == port)
        {
            return true;
        }
    }
    return false;
}
bool modem_interface::wait_for_connection(protocol type, uint16_t port, double_t timeout) const
{
    // Set deadline for timeout.
    ros::Time deadline = ros::Time::now() + ros::Duration(timeout);
    // Loop while ROS is OK.
    ros::Rate sleeper(500);
    while(ros::ok())
    {
        // Check if connection has been made.
        if(modem_interface::is_connected(type, port))
        {
            return true;
        }
        // Check for timeout.
        else if(ros::Time::now() >= deadline)
        {
            return false;
        }
        // Spin ROS to handle node callbacks.
        else
        {
            ros::spinOnce();
        }
    }
    return false;
}

// METHODS
void modem_interface::remove_duplicates(std::list<uint16_t> &a, std::list<uint16_t> &b)
{
    // Loop over A.
    for(auto a_it = a.begin(); a_it != a.end();)
    {
        bool erased = false;
        // Loop over B.
        for(auto b_it = b.begin(); b_it != b.end(); b_it++)
        {
            // Check if port matches.
            if(*a_it == *b_it)
            {
                // Match found.  Delete both, updating a_it to next position.
                a_it = a.erase(a_it);
                b.erase(b_it);
                // Mark erased as true so a_it is not iterated a second time.
                erased = true;
                // Break for loop.
                break;
            }
        }
        // Iterate a_it if no match was found.
        if(!erased)
        {
            a_it++;
        }
    }
}

// PROPERTIES
std::vector<uint16_t> modem_interface::p_active_tcp_connections() const
{
    return modem_interface::m_active_tcp_connections;
}
std::vector<uint16_t> modem_interface::p_pending_tcp_connections() const
{
    return modem_interface::m_pending_tcp_connections;
}
std::vector<uint16_t> modem_interface::p_active_udp_connections() const
{
    return modem_interface::m_active_udp_connections;
}

// CALLBACKS: Subscribers
void modem_interface::callback_active_connections(const driver_modem::ActiveConnectionsPtr &message)
{
    // Find differences between old and new connections.
    // NOTE: Use lists instead of vectors to remove elements while looping.
    // Store old connections.
    std::list<uint16_t> active_tcp_old(modem_interface::m_active_tcp_connections.begin(), modem_interface::m_active_tcp_connections.end());
    std::list<uint16_t> active_udp_old(modem_interface::m_active_udp_connections.begin(), modem_interface::m_active_udp_connections.end());
    // Store new connections.
    std::list<uint16_t> active_tcp_new(message->tcp_active.begin(), message->tcp_active.end());
    std::list<uint16_t> active_udp_new(message->udp_active.begin(), message->udp_active.end());

    // TCP:
    modem_interface::remove_duplicates(active_tcp_old, active_tcp_new);
    // Connections left in _old have been removed, while connections left in _new have been added.
    for(auto it = active_tcp_old.begin(); it != active_tcp_old.end(); it++)
    {
        // Remove TCP TX service client.
        modem_interface::m_services_send_tcp.at(*it).shutdown();
        modem_interface::m_services_send_tcp.erase(*it);
        // Remove TCP RX subscriber.
        modem_interface::m_subscribers_tcp_rx.at(*it).shutdown();
        modem_interface::m_subscribers_tcp_rx.erase(*it);
    }
    for(auto it = active_tcp_new.begin(); it != active_tcp_new.end(); it++)
    {
        // Add TCP TX service client.
        std::stringstream topic_front;
        topic_front << "tcp/" << *it;
        modem_interface::m_services_send_tcp.insert(std::make_pair(*it, modem_interface::m_node->serviceClient<driver_modem::SendTCP>(topic_front.str() + "/tx")));
        // Add TCP RX subscriber.
        modem_interface::m_subscribers_tcp_rx.insert(std::make_pair(*it, modem_interface::m_node->subscribe<driver_modem::DataPacket>(topic_front.str() + "/rx", 1, std::bind(&modem_interface::callback_tcp_rx, this, std::placeholders::_1, *it))));
    }

    // UDP:
    modem_interface::remove_duplicates(active_udp_old, active_udp_new);
    // Connections left in _old have been removed, while connections left in _new have been added.
    for(auto it = active_udp_old.begin(); it != active_udp_old.end(); it++)
    {
        // Remove UDP TX publisher.
        modem_interface::m_publishers_udp.at(*it).shutdown();
        modem_interface::m_publishers_udp.erase(*it);
        // Remove UDP RX subscriber.
        modem_interface::m_subscribers_udp_rx.at(*it).shutdown();
        modem_interface::m_subscribers_udp_rx.erase(*it);
    }
    for(auto it = active_udp_new.begin(); it != active_udp_new.end(); it++)
    {
        // Add UDP TX publisher.
        std::stringstream topic_front;
        topic_front << "udp/" << *it;
        modem_interface::m_publishers_udp.insert(std::make_pair(*it, modem_interface::m_node->advertise<driver_modem::DataPacket>(topic_front.str() + "/tx", 1)));
        // Add UDP RX subscriber.
        modem_interface::m_subscribers_udp_rx.insert(std::make_pair(*it, modem_interface::m_node->subscribe<driver_modem::DataPacket>(topic_front.str() + "/rx", 1, std::bind(&modem_interface::callback_udp_rx, this, std::placeholders::_1, *it))));
    }

    // Assign new connections to internal storage.
    modem_interface::m_pending_tcp_connections = message->tcp_pending;
    modem_interface::m_active_tcp_connections = message->tcp_active;
    modem_interface::m_active_udp_connections = message->udp_active;
}
void modem_interface::callback_tcp_rx(const driver_modem::DataPacketConstPtr &message, uint16_t port)
{
    // Forward to external callback.
    modem_interface::m_callback_tcp_rx(port, message);
}
void modem_interface::callback_udp_rx(const driver_modem::DataPacketConstPtr &message, uint16_t port)
{
    // Forward to external callback.
    modem_interface::m_callback_udp_rx(port, message);
}
