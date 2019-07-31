#include "driver.h"

// CONSTRUCTORS
driver::driver(std::string local_ip, std::string remote_ip,
               std::function<void(connection_type, uint16_t, uint8_t *, uint32_t)> rx_callback,
               std::function<void(uint16_t)> tcp_connected_callback,
               std::function<void(uint16_t)> tcp_disconnected_callback)
{
    // Create and store local/remote IP addresses.
    driver::m_local_ip = boost::asio::ip::address::from_string(local_ip);
    driver::m_remote_ip = boost::asio::ip::address::from_string(remote_ip);

    // Store local copy of callbacks.
    driver::m_callback_rx = rx_callback;
    driver::m_callback_tcp_connected = tcp_connected_callback;
    driver::m_callback_tcp_disconnected = tcp_disconnected_callback;
}
driver::~driver()
{
    // Close and delete any remaining connections.

    // Get list of pending TCP ports.
    std::vector<uint16_t> tcp_pending_ports;
    for(std::map<uint16_t, tcp_connection*>::iterator it = driver::m_tcp_pending.begin(); it != driver::m_tcp_pending.end(); it++)
    {
        tcp_pending_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < tcp_pending_ports.size(); i++)
    {
        driver::remove_connection(connection_type::TCP, tcp_pending_ports.at(i));
    }

    // Get list of active TCP ports.
    std::vector<uint16_t> tcp_active_ports;
    for(std::map<uint16_t, tcp_connection*>::iterator it = driver::m_tcp_active.begin(); it != driver::m_tcp_active.end(); it++)
    {
        tcp_active_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < tcp_active_ports.size(); i++)
    {
        driver::remove_connection(connection_type::TCP, tcp_active_ports.at(i));
    }

    // Get list of UDP ports that are open.
    std::vector<uint16_t> udp_active_ports;
    for(std::map<uint16_t, udp_connection*>::iterator it = driver::m_udp_active.begin(); it != driver::m_udp_active.end(); it++)
    {
        udp_active_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < udp_active_ports.size(); i++)
    {
        driver::remove_connection(connection_type::UDP, udp_active_ports.at(i));
    }
}

// PUBLIC METHODS: START/STOP
void driver::start()
{
    driver::m_thread = boost::thread(boost::bind(&boost::asio::io_service::run, boost::ref(driver::m_service)));
}
void driver::stop()
{
    driver::m_service.stop();
    driver::m_thread.join();
}

// PUBLIC METHODS: CONNECTION MANAGEMENT
bool driver::add_tcp_connection(tcp_connection::role role, uint16_t port)
{
    // Check if the connection already exists.
    if(driver::m_tcp_pending.count(port) == 0 && driver::m_tcp_active.count(port) == 0 && role != tcp_connection::role::UNASSIGNED)
    {
        // Create the TCP connection.
        tcp_connection* new_tcp = new tcp_connection(driver::m_service, tcp::endpoint(driver::m_local_ip, port));

        // Add the connected/disconnected/rx callbacks.
        new_tcp->attach_connected_callback(std::bind(&driver::callback_tcp_connected, this, std::placeholders::_1));
        new_tcp->attach_disconnected_callback(std::bind(&driver::callback_tcp_disconnected, this, std::placeholders::_1));
        // NOTE: rx callback is forwarded from external.
        new_tcp->attach_rx_callback(driver::m_callback_rx);

        // Add connection to pending.
        driver::m_tcp_pending.insert(std::make_pair(port, new_tcp));

        switch(role)
        {
        case tcp_connection::role::UNASSIGNED:
        {
            // This case will never occur due to if condition.
            return false;
        }
        case tcp_connection::role::SERVER:
        {
            return new_tcp->start_server();
        }
        case tcp_connection::role::CLIENT:
        {
            return new_tcp->start_client(tcp::endpoint(driver::m_remote_ip, port));
        }
        }
    }
    else
    {
        return false;
    }
}
bool driver::add_udp_connection(uint16_t port)
{
    if(driver::m_udp_active.count(port) == 0)
    {
        // Create the UDP connection.
        udp_connection* new_udp = new udp_connection(driver::m_service, udp::endpoint(driver::m_local_ip, port));
        // Attach the rx callback.
        new_udp->attach_rx_callback(driver::m_callback_rx);
        // Add connection to map.
        driver::m_udp_active.insert(std::make_pair(port, new_udp));

        return true;
    }
    else
    {
        return false;
    }
}
bool driver::remove_connection(connection_type type, uint16_t port)
{
    switch(type)
    {
    case connection_type::TCP:
    {
        // Check pending map.
        if(driver::m_tcp_pending.count(port) > 0)
        {
            // Get a pointer to the tcp connection.
            tcp_connection* tcp = driver::m_tcp_pending.at(port);

            // Stop the connection.
            // NOTE: This function causes the tcp_connection pointer to self delete.
            tcp->disconnect();

            // Remove the entry from the map.
            driver::m_tcp_pending.erase(port);

            return true;
        }
        // Check active map.
        else if(driver::m_tcp_active.count(port) > 0)
        {
            // Get a pointer to the tcp connection.
            tcp_connection* tcp = driver::m_tcp_active.at(port);

            // Stop the connection.
            // NOTE: This function causes the tcp_connection pointer to self delete.
            tcp->disconnect();

            // Remove the entry from the map.
            driver::m_tcp_active.erase(port);

            return true;
        }
        else
        {
            return false;
        }
    }
    case connection_type::UDP:
    {
        if(driver::m_udp_active.count(port) > 0)
        {
            // Get a pointer to the udp connection.
            udp_connection* udp = driver::m_udp_active.at(port);

            // Remove the entry from the map.
            driver::m_udp_active.erase(port);

            // Delete the connection.
            delete udp;

            return true;
        }
        else
        {
            return false;
        }
    }
    }
}

// PUBLIC METHODS: IO
bool driver::tx(connection_type type, uint16_t port, const uint8_t *data, uint32_t length)
{
    switch(type)
    {
    case connection_type::TCP:
    {
        if(driver::m_tcp_active.count(port) > 0)
        {
            return driver::m_tcp_active.at(port)->tx(data, length);
        }
        else
        {
            return false;
        }
    }
    case connection_type::UDP:
    {
        if(driver::m_udp_active.count(port) > 0)
        {
            driver::m_udp_active.at(port)->tx(data, length);
            return true;
        }
        else
        {
            return false;
        }
    }
    }
}

// PROPERTIES
std::vector<uint16_t> driver::p_pending_tcp_connections() const
{
    std::vector<uint16_t> output;

    for(std::map<uint16_t, tcp_connection*>::const_iterator it = driver::m_tcp_pending.cbegin(); it != driver::m_tcp_pending.cend(); it++)
    {
        output.push_back(it->first);
    }

    return output;
}
std::vector<uint16_t> driver::p_active_tcp_connections() const
{
    std::vector<uint16_t> output;

    for(std::map<uint16_t, tcp_connection*>::const_iterator it = driver::m_tcp_active.cbegin(); it != driver::m_tcp_active.cend(); it++)
    {
        output.push_back(it->first);
    }

    return output;
}
std::vector<uint16_t> driver::p_active_udp_connections() const
{
    std::vector<uint16_t> output;

    for(std::map<uint16_t, udp_connection*>::const_iterator it = driver::m_udp_active.cbegin(); it != driver::m_udp_active.cend(); it++)
    {
        output.push_back(it->first);
    }

    return output;
}

// CALLBACKS
void driver::callback_tcp_connected(uint16_t port)
{
    // Move TCP connection from pending to active map.
    if(driver::m_tcp_pending.count(port) > 0)
    {
        driver::m_tcp_active.insert(std::make_pair(port, driver::m_tcp_pending.at(port)));
        driver::m_tcp_pending.erase(port);
    }

    // Pass connected callback/signal externally.
    driver::m_callback_tcp_connected(port);
}
void driver::callback_tcp_disconnected(uint16_t port)
{
    // Remove the TCP connection from whichever map it's in.
    driver::remove_connection(connection_type::TCP, port);

    // Pass disconnect callback/signal externally.
    driver::m_callback_tcp_disconnected(port);
}
