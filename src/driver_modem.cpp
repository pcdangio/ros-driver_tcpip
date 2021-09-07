#include "driver_modem.hpp"

#include "tcp_socket.hpp"
#include "udp_socket.hpp"

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
    driver_modem_t::m_service_resolve_ip = private_node.advertiseService("resolve_ip", &driver_modem_t::service_resolve_ip, this);
    driver_modem_t::m_service_start_tcp_server = private_node.advertiseService("start_tcp_server", &driver_modem_t::service_start_tcp_server, this);
    driver_modem_t::m_service_stop_tcp_server = private_node.advertiseService("stop_tcp_server", &driver_modem_t::service_stop_tcp_server, this);
    driver_modem_t::m_service_open_tcp_socket = private_node.advertiseService("open_tcp_socket", &driver_modem_t::service_open_tcp_socket, this);
    driver_modem_t::m_service_open_udp_socket = private_node.advertiseService("open_udp_socket", &driver_modem_t::service_open_udp_socket, this);
    driver_modem_t::m_service_close_socket = private_node.advertiseService("close_socket", &driver_modem_t::service_close_socket, this);

    // Log initialization.
    ROS_INFO("initialized successfully");
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
    // Create rate for spinning.
    ros::Rate loop_rate(100);

    // Create IO service work instance to keep io_service alive while in scope.
    boost::asio::io_service::work io_service_work(driver_modem_t::m_io_service);

    // Process ROS and ASIO until node shuts down.
    while(ros::ok())
    {
        // Spin ASIO.
        driver_modem_t::m_io_service.poll();

        // Spin ROS.
        ros::spinOnce();

        // Clean up any self-closed sockets (e.g. TCP disconnectes)
        bool sockets_updated = false;
        auto socket = driver_modem_t::m_sockets.begin();
        while(socket != driver_modem_t::m_sockets.end())
        {
            // Check if socket is open.
            if(!socket->second->is_open())
            {
                // Delete socket instance.
                delete socket->second;
                // Remove from map.
                socket = driver_modem_t::m_sockets.erase(socket);
                // Indicate that status should be updated.
                sockets_updated = true;
            }
            else
            {
                ++socket;
            }                       
        }
        // If sockets updated, publish new status.
        if(sockets_updated)
        {
            driver_modem_t::publish_status();
        }

        // Sleep for remainder of loop.
        loop_rate.sleep();
    }
}

// SERVICE CALLBACKS
bool driver_modem_t::service_resolve_ip(driver_modem_msgs::resolve_ipRequest& request, driver_modem_msgs::resolve_ipResponse& response)
{
    // Create a resolver query.
    boost::asio::ip::udp::resolver::query query(request.hostname, "");

    // Create the resolver.
    boost::asio::ip::udp::resolver resolver(driver_modem_t::m_io_service);

    // Attempt to resolve the hostname.
    boost::system::error_code error;
    auto result = resolver.resolve(query, error);
    if(error)
    {
        // Indicate that resolving has failed.
        ROS_ERROR_STREAM("failed to resolve hostname " << request.hostname << " (" << error.message() << ")");
        return false;
    }

    // Populate response with resolved IP.
    memcpy(response.ip.data(), result->endpoint().address().to_v4().to_bytes().data(), 4);
    
    // Indicate success.
    ROS_INFO_STREAM("resolved hostname " << request.hostname << " to " << result->endpoint().address().to_string());
    return true;
}
bool driver_modem_t::service_start_tcp_server(driver_modem_msgs::start_tcp_serverRequest& request, driver_modem_msgs::start_tcp_serverResponse& response)
{
    // Get unique ID.
    uint32_t id = 0;
    while(driver_modem_t::m_tcp_servers.count(id))
    {
        id++;
    }

    // Create the new TCP server.
    tcp_server_t* tcp_server = new tcp_server_t(driver_modem_t::m_io_service, id, std::bind(&driver_modem_t::tcp_connection, this, std::placeholders::_1, std::placeholders::_2));

    // Attempt to start the TCP server on the requested endpoint.
    if(tcp_server->start(request.local_endpoint))
    {
        // Add the server to the map.
        driver_modem_t::m_tcp_servers[id] = tcp_server;
        // Populate the server response.
        response.server_id = id;

        // Publish updated status.
        driver_modem_t::publish_status();

        // Indicate success.
        return true;
    }
    else
    {
        // Delete the server.
        delete tcp_server;
        // Indicate failure.
        return false;
    }
}
bool driver_modem_t::service_stop_tcp_server(driver_modem_msgs::stop_tcp_serverRequest& request, driver_modem_msgs::stop_tcp_serverResponse& response)
{
    // Find the tcp server by ID.
    auto iterator = driver_modem_t::m_tcp_servers.find(request.server_id);
    if(iterator != driver_modem_t::m_tcp_servers.end())
    {
        // Stop the server.
        iterator->second->stop();
        // Delete server instance.
        delete iterator->second;
        // Delete from map.
        driver_modem_t::m_tcp_servers.erase(iterator);

        // Publish updated status.
        driver_modem_t::publish_status();

        // Indicate success.
        return true;
    }
    else
    {
        // Indicate error.
        ROS_ERROR_STREAM("failed to remove tcp server " << request.server_id << " (does not exist)");

        // Indicate failure.
        return false;
    }
}
bool driver_modem_t::service_open_tcp_socket(driver_modem_msgs::open_tcp_socketRequest& request, driver_modem_msgs::open_tcp_socketResponse& response)
{
    // Get unique ID.
    uint32_t id = 0;
    while(driver_modem_t::m_sockets.count(id))
    {
        id++;
    }

    // Create the TCP socket.
    tcp_socket_t* tcp_socket = new tcp_socket_t(driver_modem_t::m_io_service, id);

    // Attempt to connect to the remote endpoint.
    if(tcp_socket->connect(request.local_endpoint, request.remote_endpoint))
    {
        // Connection succeeded.

        // Add socket to map.
        driver_modem_t::m_sockets[id] = tcp_socket;
        
        // Publish updated status.
        driver_modem_t::publish_status();

        // Populate response.
        response.socket_id = id;
        
        // Indicate success.
        return true;
    }
    else
    {
        // Delete socket.
        delete tcp_socket;
        // indicate failure.
        return false;
    }
}
bool driver_modem_t::service_open_udp_socket(driver_modem_msgs::open_udp_socketRequest& request, driver_modem_msgs::open_udp_socketResponse& response)
{
    // Get unique ID.
    uint32_t id = 0;
    while(driver_modem_t::m_sockets.count(id))
    {
        id++;
    }

    // Create the UDP socket.
    udp_socket_t* udp_socket = new udp_socket_t(driver_modem_t::m_io_service, id);

    // Attempt to open the socket.
    if(udp_socket->open(request.local_endpoint))
    {
        // Open succeeded.

        // Add socket to map.
        driver_modem_t::m_sockets[id] = udp_socket;
        
        // Publish updated status.
        driver_modem_t::publish_status();

        // Populate response.
        response.socket_id = id;
        
        // Indicate success.
        return true;
    }
    else
    {
        // Delete socket.
        delete udp_socket;
        // Indicate failure.
        return false;
    }
}
bool driver_modem_t::service_close_socket(driver_modem_msgs::close_socketRequest& request, driver_modem_msgs::close_socketResponse& response)
{
    // Find the requested socket.
    auto iterator = driver_modem_t::m_sockets.find(request.socket_id);
    if(iterator != driver_modem_t::m_sockets.end())
    {
        // Close the socket.
        iterator->second->close();
        // Delete the socket instance.
        delete iterator->second;
        // Remove entry from map.
        driver_modem_t::m_sockets.erase(iterator);

        // Publish updated status.
        driver_modem_t::publish_status();

        // Indicate success.
        return true;
    }
    else
    {
        // Requested socket does not exist.
        ROS_ERROR_STREAM("failed to close socket " << request.socket_id << " (socket does not exist)");
        // Indicate failure.
        return false;
    }
}

// PUBLISHING
void driver_modem_t::publish_status() const
{
    // Create status message to publish.
    driver_modem_msgs::status message;

    // Populate TCP servers.
    for(auto server = driver_modem_t::m_tcp_servers.cbegin(); server != driver_modem_t::m_tcp_servers.cend(); ++server)
    {
        message.tcp_servers.push_back(server->second->description());
    }

    // Populate TCP and UDP sockets.
    for(auto socket = driver_modem_t::m_sockets.cbegin(); socket != driver_modem_t::m_sockets.cend(); ++socket)
    {
        // Get the socket protocol type.
        switch(socket->second->protocol())
        {
            case protocol_t::TCP:
            {
                // Convert to TCP socket.
                tcp_socket_t* tcp_socket = reinterpret_cast<tcp_socket_t*>(socket->second);
                // Add description.
                message.tcp_sockets.push_back(tcp_socket->description());
                break;
            }
            case protocol_t::UDP:
            {
                // Convert to UDP socket.
                udp_socket_t* udp_socket = reinterpret_cast<udp_socket_t*>(socket->second);
                // Add description.
                message.udp_sockets.push_back(udp_socket->description());
                break;
            }
        }
    }

    // Publish message.
    driver_modem_t::m_publisher_status.publish(message);
}

// CONNECTION
void driver_modem_t::tcp_connection(uint32_t id, boost::asio::ip::tcp::socket* socket)
{
    // Get unique ID.
    uint32_t socket_id = 0;
    while(driver_modem_t::m_sockets.count(socket_id))
    {
        socket_id++;
    }

    // Create a new tcp_socket_t from the ASIO socket and add it to the map.
    driver_modem_t::m_sockets[socket_id] = new tcp_socket_t(socket, socket_id);

    // Log new connection.
    ROS_INFO_STREAM("tcp server " << id << " accepted new connection on tcp socket " << socket_id);

    // Publish updated status.
    driver_modem_t::publish_status();
}