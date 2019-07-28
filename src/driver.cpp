#include "driver.h"

driver::driver(std::string local_ip, std::string remote_ip, std::function<void(connection_type, uint16_t, uint8_t *, uint32_t)> rx_callback, std::function<void(connection_type, uint16_t)> disconnect_callback)
{
    // Create and store local/remote IP addresses.
    driver::m_local_ip = boost::asio::ip::address::from_string(local_ip);
    driver::m_remote_ip = boost::asio::ip::address::from_string(remote_ip);

    // Store local copy of rx/disconnect callbacks.
    driver::m_rx_callback = rx_callback;
    driver::m_disconnect_callback = disconnect_callback;
}
driver::~driver()
{
    // Close and delete any remaining connections.

    // Get list of TCP ports that are open.
    std::vector<uint16_t> tcp_ports;
    for(std::map<uint16_t, tcp_connection*>::iterator it = driver::m_tcp_connections.begin(); it != driver::m_tcp_connections.end(); it++)
    {
        tcp_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < tcp_ports.size(); i++)
    {
        driver::remove_connection(connection_type::TCP, tcp_ports.at(i));
    }

    // Get list of UDP ports that are open.
    std::vector<uint16_t> udp_ports;
    for(std::map<uint16_t, udp_connection*>::iterator it = driver::m_udp_connections.begin(); it != driver::m_udp_connections.end(); it++)
    {
        udp_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < udp_ports.size(); i++)
    {
        driver::remove_connection(connection_type::UDP, udp_ports.at(i));
    }
}

bool driver::add_connection(connection_type type, uint16_t local_port, uint16_t remote_port)
{
    switch(type)
    {
    case connection_type::TCP:
    {
        // Check if the connection already exists.
        if(driver::m_tcp_connections.count(local_port) == 0)
        {
            // Create the TCP connection.
            tcp_connection* new_tcp = new tcp_connection(driver::m_service, tcp::endpoint(driver::m_local_ip, local_port), tcp::endpoint(driver::m_remote_ip, remote_port));
            // Try to connect.
            if(new_tcp->connect())
            {
                // Attach the rx callback.
                new_tcp->attach_rx_callback(driver::m_rx_callback);
                // Attach the internal disconnect callback.
                new_tcp->attach_disconnect_callback(std::bind(&driver::disconnect_callback, this, std::placeholders::_1));
                // Add connection to map.
                driver::m_tcp_connections.insert(std::make_pair(local_port, new_tcp));

                return true;
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
    case connection_type::UDP:
    {
        if(driver::m_udp_connections.count(local_port) == 0)
        {
            // Create the UDP connection.
            udp_connection* new_udp = new udp_connection(driver::m_service, udp::endpoint(driver::m_local_ip, local_port), udp::endpoint(driver::m_remote_ip, remote_port));
            // Attach the rx callback.
            new_udp->attach_rx_callback(driver::m_rx_callback);
            // Add connection to map.
            driver::m_udp_connections.insert(std::make_pair(local_port, new_udp));

            return true;
        }
        else
        {
            return false;
        }
    }
    }
}
bool driver::remove_connection(connection_type type, uint16_t local_port)
{
    switch(type)
    {
    case connection_type::TCP:
    {
        if(driver::m_tcp_connections.count(local_port) > 0)
        {
            // Get a pointer to the tcp connection.
            tcp_connection* tcp = driver::m_tcp_connections.at(local_port);

            // Remove the entry from the map.
            driver::m_tcp_connections.erase(local_port);

            // Delete the connection.
            delete tcp;

            // Raise the external disconnect callback.
            driver::m_disconnect_callback(type, local_port);

            return true;
        }
        else
        {
            return false;
        }
    }
    case connection_type::UDP:
    {
        if(driver::m_udp_connections.count(local_port) > 0)
        {
            // Get a pointer to the udp connection.
            udp_connection* udp = driver::m_udp_connections.at(local_port);

            // Remove the entry from the map.
            driver::m_udp_connections.erase(local_port);

            // Delete the connection.
            delete udp;

            // Raise the external disconnect callback.
            driver::m_disconnect_callback(type, local_port);

            return true;
        }
        else
        {
            return false;
        }
    }
    }
}

void driver::spin_once()
{
    driver::m_service.run_one();
}

std::vector<std::pair<connection_type, uint16_t>> driver::p_connections()
{
    std::vector<std::pair<connection_type, uint16_t>> output;

    // Iterate over connections and build output list.
    for(std::map<uint16_t, tcp_connection*>::iterator it = driver::m_tcp_connections.begin(); it != driver::m_tcp_connections.end(); it++)
    {
        output.push_back(std::make_pair(connection_type::TCP, it->first));
    }
    for(std::map<uint16_t, udp_connection*>::iterator it = driver::m_udp_connections.begin(); it != driver::m_udp_connections.end(); it++)
    {
        output.push_back(std::make_pair(connection_type::UDP, it->first));
    }

    return output;
}

void driver::disconnect_callback(uint16_t local_port)
{
    // One of the tcp connections has disconnected.

    // Remove the connection from the map.
    driver::remove_connection(connection_type::TCP, local_port);
}
