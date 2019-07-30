#include "ros_node.h"

#include <driver_modem/ActiveConnections.h>

// CONSTRUCTORS
ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_modem");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle("~");

    // Read standard parameters.
    std::string param_local_ip;
    ros_node::m_node->param<std::string>("local_ip", param_local_ip, "192.168.1.2");
    std::string param_remote_ip;
    ros_node::m_node->param<std::string>("remote_ip", param_remote_ip, "192.168.1.3");

    // Read connect port parameters.
    std::vector<int> param_tcp_server_ports;
    ros_node::m_node->getParam("tcp_server_ports", param_tcp_server_ports);
    std::vector<int> param_tcp_client_ports;
    ros_node::m_node->getParam("tcp_client_ports", param_tcp_client_ports);
    std::vector<int> param_udp_ports;
    ros_node::m_node->getParam("udp_ports", param_udp_ports);

    // Initialize driver.
    ros_node::m_driver = new driver(param_local_ip,
                                    param_remote_ip,
                                    std::bind(&ros_node::callback_rx, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
                                    std::bind(&ros_node::callback_disconnected, this, std::placeholders::_1, std::placeholders::_2));

    // Set up active connections publisher.
    // This will publish each time the connections are modified.
    // Use latching so new nodes always have the latest information.
    ros_node::m_publisher_active_connections = ros_node::m_node->advertise<driver_modem::ActiveConnections>("active_connections", 1, true);

    // Set up services for adding/removing connections.
    ros_node::m_service_add_tcp_connection = ros_node::m_node->advertiseService("add_tcp_connection", &ros_node::service_add_tcp_connection, this);
    ros_node::m_service_add_udp_connection = ros_node::m_node->advertiseService("add_udp_connection", &ros_node::service_add_udp_connection, this);
    ros_node::m_service_remove_tcp_connection = ros_node::m_node->advertiseService<driver_modem::RemoveConnectionRequest, driver_modem::RemoveConnectionResponse>("remove_tcp_connection", std::bind(&ros_node::service_remove_connection, this, std::placeholders::_1, std::placeholders::_2, connection_type::TCP));
    ros_node::m_service_remove_udp_connection = ros_node::m_node->advertiseService<driver_modem::RemoveConnectionRequest, driver_modem::RemoveConnectionResponse>("remove_udp_connection", std::bind(&ros_node::service_remove_connection, this, std::placeholders::_1, std::placeholders::_2, connection_type::UDP));

    // Set up tx/rx publishers, subscribers, and services.

    // TCP Servers:
    for(uint32_t i = 0; i < param_tcp_server_ports.size(); i++)
    {
        // Get port from vector.
        uint16_t port = static_cast<uint16_t>(param_tcp_server_ports.at(i));

        // Add connection to node.
        ros_node::add_tcp_connection(tcp_connection::role::SERVER, port);
    }

    // TCP Clients:
    for(uint32_t i = 0; i < param_tcp_client_ports.size(); i++)
    {
        // Get port from vector.
        uint16_t port = static_cast<uint16_t>(param_tcp_client_ports.at(i));

        // Add connection to node.
        ros_node::add_tcp_connection(tcp_connection::role::CLIENT, port);
    }

    // UDP:
    for(uint32_t i = 0; i < param_udp_ports.size(); i++)
    {
        // Get local/remote ports from vectors.
        uint16_t port = static_cast<uint16_t>(param_udp_ports.at(i));

        // Add connection to node.
        ros_node::add_udp_connection(port);
    }

    ROS_INFO_STREAM("Modem initialized.");
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// METHODS
void ros_node::spin()
{
    // Start the driver thread.
    ros_node::m_driver->start();

    // Spin ROS node.
    ros::spin();

    // Stop the driver thread.
    ros_node::m_driver->stop();
}
bool ros_node::add_tcp_connection(tcp_connection::role role, uint16_t port)
{
    // Add connection to driver.
    if(ros_node::m_driver->add_tcp_connection(role, port))
    {
        // RX Publisher:
        // Generate topic name.
        std::stringstream rx_topic;
        rx_topic << "tcp/" << port << "/rx";
        // Add new rx publisher to the map.
        ros_node::m_tcp_rx.insert(std::make_pair(port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

        // TX Service:
        // Generate topic name.
        std::stringstream tx_topic;
        tx_topic << "tcp/" << port << "/tx";
        // Add new tx service server to the map.
        ros_node::m_tcp_tx.insert(std::make_pair(port, ros_node::m_node->advertiseService<driver_modem::SendTCPRequest, driver_modem::SendTCPResponse>(tx_topic.str(), std::bind(&ros_node::service_tcp_tx, this, std::placeholders::_1, std::placeholders::_2, port))));

        // Publish updated connections.
        ros_node::publish_active_connections();

        return true;
    }
    else
    {
        return false;
    }
}
bool ros_node::add_udp_connection(uint16_t port)
{
    // Add connection to driver.
    if(ros_node::m_driver->add_udp_connection(port, port))
    {
        // RX Publisher:
        // Generate topic name.
        std::stringstream rx_topic;
        rx_topic << "udp/" << port << "/rx";
        // Add new rx publisher to the map.
        ros_node::m_udp_rx.insert(std::make_pair(port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

        // TX Subscriber:
        // Generate topic name.
        std::stringstream tx_topic;
        tx_topic << "udp/" << port << "/tx";
        // Add new tx subscriber to the map.
        ros_node::m_udp_tx.insert(std::make_pair(port, ros_node::m_node->subscribe<driver_modem::DataPacket>(tx_topic.str(), 1, std::bind(&ros_node::callback_udp_tx, this, std::placeholders::_1, port))));

        // Publish updated connections.
        ros_node::publish_active_connections();

        return true;
    }
    else
    {
        return false;
    }
}
bool ros_node::remove_connection(connection_type type, uint16_t port)
{
    // Remove the connection from the driver.
    if(ros_node::m_driver->remove_connection(type, port))
    {
        // Remove publishers/subscribers/callbacks
        switch(type)
        {
        case connection_type::TCP:
        {
            // Remove RX publisher
            if(ros_node::m_tcp_rx.count(port) > 0)
            {
                // Cancel topic.
                ros_node::m_tcp_rx.at(port).shutdown();
                // Remove from map.
                ros_node::m_tcp_rx.erase(port);
            }
            // Remove TX service
            if(ros_node::m_tcp_tx.count(port) > 0)
            {
                // Cancel service.
                ros_node::m_tcp_tx.at(port).shutdown();
                // Remove from map.
                ros_node::m_tcp_tx.erase(port);
            }
            break;
        }
        case connection_type::UDP:
        {
            // Remove RX publisher
            if(ros_node::m_udp_rx.count(port) > 0)
            {
                // Cancel topic.
                ros_node::m_udp_rx.at(port).shutdown();
                // Remove from map.
                ros_node::m_udp_rx.erase(port);
            }
            // Remove TX subscriber
            if(ros_node::m_udp_tx.count(port) > 0)
            {
                // Cancel service.
                ros_node::m_udp_tx.at(port).shutdown();
                // Remove from map.
                ros_node::m_udp_tx.erase(port);
            }
            break;
        }
        }

        // Publish updated connections.
        ros_node::publish_active_connections();

        return true;
    }
    else
    {
        return false;
    }
}
void ros_node::publish_active_connections()
{
    // Convert current connections into ActiveConnections message.

    // Create output message.
    driver_modem::ActiveConnections message;
    message.header.stamp = ros::Time::now();

    // Get the list of active connections from the driver.
    std::vector<uint16_t> pending_tcp = ros_node::m_driver->p_pending_tcp_connections();
    std::vector<uint16_t> active_tcp = ros_node::m_driver->p_active_tcp_connections();
    std::vector<uint16_t> active_udp = ros_node::m_driver->p_active_udp_connections();

    // Iterate over each connection type to populate message.
    for(uint32_t i = 0; i < pending_tcp.size(); i++)
    {
        message.tcp_pending.push_back(pending_tcp.at(i));
    }
    for(uint32_t i = 0; i < active_tcp.size(); i++)
    {
        message.tcp_active.push_back(active_tcp.at(i));
    }
    for(uint32_t i = 0; i < active_udp.size(); i++)
    {
        message.udp_active.push_back(active_udp.at(i));
    }

    // Publish the message.
    ros_node::m_publisher_active_connections.publish(message);
}

// MESSAGE CALLBACKS
void ros_node::callback_udp_tx(const driver_modem::DataPacketConstPtr &message, uint16_t port)
{
    ros_node::m_driver->tx(connection_type::UDP, port, message->data.data(), static_cast<uint32_t>(message->data.size()));
}

// SERVICE CALLBACKS
bool ros_node::service_add_tcp_connection(driver_modem::AddTCPConnectionRequest& request, driver_modem::AddTCPConnectionResponse& response)
{
    bool result = ros_node::add_tcp_connection(static_cast<tcp_connection::role>(request.role), request.port);
    response.success = result;

    return result;
}
bool ros_node::service_add_udp_connection(driver_modem::AddUDPConnectionRequest& request, driver_modem::AddUDPConnectionResponse& response)
{
    bool result = ros_node::add_udp_connection(request.port);
    response.success = result;

    return result;
}
bool ros_node::service_remove_connection(driver_modem::RemoveConnectionRequest& request, driver_modem::RemoveConnectionResponse& response, connection_type type)
{
    bool result = ros_node::remove_connection(static_cast<connection_type>(request.protocol), request.port);
    response.success = result;

    return result;
}
bool ros_node::service_tcp_tx(driver_modem::SendTCPRequest &request, driver_modem::SendTCPResponse &response, uint16_t port)
{
    bool result = ros_node::m_driver->tx(connection_type::TCP, port, request.packet.data.data(), static_cast<uint32_t>(request.packet.data.size()));
    response.success = result;
    return result;
}

// DRIVER CALLBACKS
void ros_node::callback_rx(connection_type type, uint16_t port, uint8_t *data, uint32_t length)
{
    // Deep copy data into new DataPacket message.
    driver_modem::DataPacket message;
    message.header.stamp = ros::Time::now();
    for(uint32_t i = 0; i < length; i++)
    {
        message.data.push_back(data[i]);
    }
    // Clear raw data array.
    delete [] data;

    // Publish received message.
    switch(type)
    {
    case connection_type::TCP:
    {
        ros_node::m_tcp_rx.at(port).publish(message);
        break;
    }
    case connection_type::UDP:
    {
        ros_node::m_udp_rx.at(port).publish(message);
        break;
    }
    }
}
void ros_node::callback_disconnected(connection_type type, uint16_t port)
{
    // Remove the associated topic/service.
    ros_node::remove_connection(type, port);
}
