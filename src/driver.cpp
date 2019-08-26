#include "driver.h"

#include <stdexcept>

// CONSTRUCTORS
driver::driver(std::string local_ip, std::string remote_host,
               std::function<void(protocol, uint16_t, uint8_t *, uint32_t, address)> rx_callback,
               std::function<void(uint16_t)> tcp_connected_callback,
               std::function<void(uint16_t)> tcp_disconnected_callback)
{    
    // Create and store local ip.
    driver::m_local_ip = boost::asio::ip::address::from_string(local_ip);

    // Try to resolve and set remote host.
    if(driver::set_remote_host(remote_host) == false)
    {
        std::stringstream message;
        message << "Could not resolve remote host: " << remote_host;
        throw std::runtime_error(message.str());
    }

    // Store local copy of callbacks.
    driver::m_callback_rx = rx_callback;
    driver::m_callback_tcp_connected = tcp_connected_callback;
    driver::m_callback_tcp_disconnected = tcp_disconnected_callback;
}
driver::~driver()
{
    // Close and delete any remaining connections.
    driver::remove_all_connections();

    // Delete io service worker.
    if(driver::m_service_work)
    {
        delete driver::m_service_work;
    }
}

// PUBLIC METHODS: START/STOP
void driver::start()
{
    // Create worker object to keep io service running.
    driver::m_service_work = new boost::asio::io_service::work(driver::m_service);
    // Run io service in separate thread.
    driver::m_thread = boost::thread(boost::bind(&boost::asio::io_service::run, boost::ref(driver::m_service)));
}
void driver::stop()
{
    // Stop the service.
    driver::m_service.stop();

    // Join the thread.
    driver::m_thread.join();

    // Delete the service worker.
    delete driver::m_service_work;
    driver::m_service_work = nullptr;
}

// PUBLIC METHODS: CONNECTION MANAGEMENT
bool driver::set_remote_host(std::string remote_host)
{
    // Resolve and store remote ip.
    // Can just use UDP resolver here.
    udp::resolver::query query(remote_host, "");
    udp::resolver resolver(driver::m_service);
    try
    {
        // Resolve IP Address
        driver::m_remote_ip = resolver.resolve(query)->endpoint().address();

        // Close all active connections.
        driver::remove_all_connections();

        return true;
    }
    catch(...)
    {
        return false;
    }
}
bool driver::add_tcp_connection(tcp_role role, uint16_t port)
{
    // Make sure role is valid.
    if(role != tcp_role::UNASSIGNED)
    {
        // Check if the connection already exists.
        if(driver::m_tcp_pending.count(port) == 0 && driver::m_tcp_active.count(port) == 0)
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
            case tcp_role::UNASSIGNED:
            {
                // This case will never occur due to if condition.
                return false;
            }
            case tcp_role::SERVER:
            {
                return new_tcp->start_server();
            }
            case tcp_role::CLIENT:
            {
                return new_tcp->start_client(tcp::endpoint(driver::m_remote_ip, port));
            }
            }
        }
        else
        {
            // The connection is already active or pending. Check if it's role matches the requested role.
            if(driver::m_tcp_active.count(port))
            {
                return driver::m_tcp_active.at(port)->p_role() == role;
            }
            else
            {
                return driver::m_tcp_pending.at(port)->p_role() == role;
            }
        }
    }
    else
    {
        /// Connection role is unassigned.
        return false;
    }
}
bool driver::add_udp_connection(uint16_t port)
{
    if(driver::m_udp_active.count(port) == 0)
    {
        // Create the UDP connection.
        udp_connection* new_udp = new udp_connection(driver::m_service, udp::endpoint(driver::m_local_ip, port), udp::endpoint(driver::m_remote_ip, port));
        // Attach the rx callback.
        new_udp->attach_rx_callback(driver::m_callback_rx);
        // Add connection to map.
        driver::m_udp_active.insert(std::make_pair(port, new_udp));

        return true;
    }
    else
    {
        // Return true since the connection already exists.
        return true;
    }
}
bool driver::remove_connection(protocol type, uint16_t port)
{
    switch(type)
    {
    case protocol::TCP:
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
            // Connection already doesn't exist.
            return true;
        }
    }
    case protocol::UDP:
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
            // Connection already doesn't exist.
            return true;
        }
    }
    }
}
void driver::remove_all_connections()
{
    // Get list of pending TCP ports.
    std::vector<uint16_t> tcp_pending_ports;
    for(std::map<uint16_t, tcp_connection*>::iterator it = driver::m_tcp_pending.begin(); it != driver::m_tcp_pending.end(); it++)
    {
        tcp_pending_ports.push_back(it->first);
    }
    // Remove each connection.
    for(uint32_t i = 0; i < tcp_pending_ports.size(); i++)
    {
        driver::remove_connection(protocol::TCP, tcp_pending_ports.at(i));
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
        driver::remove_connection(protocol::TCP, tcp_active_ports.at(i));
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
        driver::remove_connection(protocol::UDP, udp_active_ports.at(i));
    }
}

// PUBLIC METHODS: IO
bool driver::tx(protocol type, uint16_t port, const uint8_t *data, uint32_t length)
{
    switch(type)
    {
    case protocol::TCP:
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
    case protocol::UDP:
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

// PUBLIC METHODS: STATIC
std::string driver::protocol_string(protocol value)
{
    switch(value)
    {
    case protocol::TCP:
        return "TCP";
    case protocol::UDP:
        return "UDP";
    }
}
std::string driver::tcp_role_string(tcp_role value)
{
    switch(value)
    {
    case tcp_role::UNASSIGNED:
        return "Unassigned";
    case tcp_role::CLIENT:
        return "Client";
    case tcp_role::SERVER:
        return "Server";
    }
}

// PROPERTIES
std::string driver::p_remote_host()
{
    return driver::m_remote_ip.to_string();
}
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
    driver::remove_connection(protocol::TCP, port);

    // Pass disconnect callback/signal externally.
    driver::m_callback_tcp_disconnected(port);
}
