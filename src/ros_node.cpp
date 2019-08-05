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
    std::string param_remote_host;
    ros_node::m_node->param<std::string>("remote_host", param_remote_host, "192.168.1.3");

    // Read connect port parameters.
    std::vector<int> param_tcp_server_ports;
    ros_node::m_node->getParam("tcp_server_ports", param_tcp_server_ports);
    std::vector<int> param_tcp_client_ports;
    ros_node::m_node->getParam("tcp_client_ports", param_tcp_client_ports);
    std::vector<int> param_udp_ports;
    ros_node::m_node->getParam("udp_ports", param_udp_ports);

    // Initialize driver.
    try
    {
        ros_node::m_driver = new driver(param_local_ip,
                                        param_remote_host,
                                        std::bind(&ros_node::callback_rx, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
                                        std::bind(&ros_node::callback_tcp_connected, this, std::placeholders::_1),
                                        std::bind(&ros_node::callback_tcp_disconnected, this, std::placeholders::_1));
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        delete ros_node::m_node;
        exit(1);
    }


    // Set up active connections publisher.
    // This will publish each time the connections are modified.
    // Use latching so new nodes always have the latest information.
    ros_node::m_publisher_active_connections = ros_node::m_node->advertise<driver_modem::ActiveConnections>("active_connections", 1, true);

    // Set up service for setting/getting remote host.
    ros_node::m_service_set_remote_host = ros_node::m_node->advertiseService("set_remote_host", &ros_node::service_set_remote_host, this);
    ros_node::m_service_get_remote_host = ros_node::m_node->advertiseService("get_remote_host", &ros_node::service_get_remote_host, this);

    // Set up services for adding/removing connections.
    ros_node::m_service_add_tcp_connection = ros_node::m_node->advertiseService("add_tcp_connection", &ros_node::service_add_tcp_connection, this);
    ros_node::m_service_add_udp_connection = ros_node::m_node->advertiseService("add_udp_connection", &ros_node::service_add_udp_connection, this);
    ros_node::m_service_remove_connection = ros_node::m_node->advertiseService("remove_connection", &ros_node::service_remove_connection, this);

    // Set up tx/rx publishers, subscribers, and services.

    // TCP Servers:
    for(uint32_t i = 0; i < param_tcp_server_ports.size(); i++)
    {
        // Get port from vector.
        uint16_t port = static_cast<uint16_t>(param_tcp_server_ports.at(i));

        // Add connection to node.
        ros_node::add_tcp_connection(tcp_role::SERVER, port, false);
    }

    // TCP Clients:
    for(uint32_t i = 0; i < param_tcp_client_ports.size(); i++)
    {
        // Get port from vector.
        uint16_t port = static_cast<uint16_t>(param_tcp_client_ports.at(i));

        // Add connection to node.
        ros_node::add_tcp_connection(tcp_role::CLIENT, port, false);
    }

    // UDP:
    for(uint32_t i = 0; i < param_udp_ports.size(); i++)
    {
        // Get local/remote ports from vectors.
        uint16_t port = static_cast<uint16_t>(param_udp_ports.at(i));

        // Add connection to node.
        ros_node::add_udp_connection(port, false);
    }

    // Manually publish connections after group add.
    ros_node::publish_active_connections();

    ROS_INFO_STREAM("Modem initialized with local IP: " << param_local_ip << " and remote host: " << param_remote_host);
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// PUBLIC METHODS
void ros_node::spin()
{
    // Start the driver thread.
    ros_node::m_driver->start();

    // Spin ROS node.
    ros::spin();

    // Stop the driver thread.
    ros_node::m_driver->stop();
}

// PRIVATE METHODS: CONNECTION MANAGEMENT
bool ros_node::add_tcp_connection(tcp_role role, uint16_t port, bool publish_connections)
{
    if(ros_node::m_driver->add_tcp_connection(role, port) && publish_connections)
    {
        // Publish active connections, since connection will initially be in PENDING status.
        ros_node::publish_active_connections();

        // NOTE: TCP will update topics once becoming active through signal on connected tcp callback.

        return true;
    }
    else
    {
        return false;
    }
}
bool ros_node::add_udp_connection(uint16_t port, bool publish_connections)
{
    if(ros_node::m_driver->add_udp_connection(port) && publish_connections)
    {
        // Add UDP topic.
        ros_node::add_connection_topics(protocol::UDP, port);

        // Publish active connections.
        ros_node::publish_active_connections();

        return true;
    }
    else
    {
        return false;
    }
}
bool ros_node::remove_connection(protocol type, uint16_t port, bool publish_connections)
{
    // Instruct driver to remove connection.
    if(ros_node::m_driver->remove_connection(type, port))
    {
        // Remove topics.
        // NOTE: For TCP, driver will not generate disconnected callbacks when driver::remove_connection() is called.
        ros_node::remove_connection_topics(type, port);

        if(publish_connections)
        {
            // Publish active connections.
            ros_node::publish_active_connections();
        }

        return true;
    }
    else
    {
        return false;
    }
}

// PRIVATE METHODS: TOPIC MANAGEMENT
void ros_node::add_connection_topics(protocol type, uint16_t port)
{
    switch(type)
    {
    case protocol::TCP:
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

        break;
    }
    case protocol::UDP:
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

        break;
    }
    }
}
void ros_node::remove_connection_topics(protocol type, uint16_t port)
{
    // Remove publishers/subscribers/callbacks of the connection.
    switch(type)
    {
    case protocol::TCP:
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
    case protocol::UDP:
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
}

// PRIVATE METHODS: MISC
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

// CALLBACKS: DRIVER
void ros_node::callback_tcp_connected(uint16_t port)
{
    // TCP has transitioned from pending to active.

    // Add the associated topic/service.
    ros_node::add_connection_topics(protocol::TCP, port);

    // Publish updated connections.
    ros_node::publish_active_connections();

    ROS_INFO_STREAM("TCP:" << port << " connected.");
}
void ros_node::callback_tcp_disconnected(uint16_t port)
{
    // Remove the associated topic/service.  Driver has already internally removed connection.
    ros_node::remove_connection_topics(protocol::TCP, port);

    // Publish updated connections.
    ros_node::publish_active_connections();

    ROS_INFO_STREAM("TCP:" << port << " disconnected.");
}
void ros_node::callback_rx(protocol type, uint16_t port, uint8_t *data, uint32_t length, address source)
{
    // Deep copy data into new DataPacket message.
    driver_modem::DataPacket message;
    message.header.stamp = ros::Time::now();
    message.source_ip = source.to_string();
    for(uint32_t i = 0; i < length; i++)
    {
        message.data.push_back(data[i]);
    }
    // Clear raw data array.
    delete [] data;

    // Publish received message.
    switch(type)
    {
    case protocol::TCP:
    {
        ros_node::m_tcp_rx.at(port).publish(message);
        break;
    }
    case protocol::UDP:
    {
        ros_node::m_udp_rx.at(port).publish(message);
        break;
    }
    }
}

// CALLBACKS: SUBSCRIBERS
void ros_node::callback_udp_tx(const driver_modem::DataPacketConstPtr &message, uint16_t port)
{
    ros_node::m_driver->tx(protocol::UDP, port, message->data.data(), static_cast<uint32_t>(message->data.size()));
}

// CALLBACKS: SERVICES
bool ros_node::service_set_remote_host(driver_modem::SetRemoteHostRequest &request, driver_modem::SetRemoteHostResponse &response)
{
    response.success = ros_node::m_driver->set_remote_host(request.remote_host);

    // Publish active connections, since changing remote host clears all connections.
    ros_node::publish_active_connections();

    return true;
}
bool ros_node::service_get_remote_host(driver_modem::GetRemoteHostRequest &request, driver_modem::GetRemoteHostResponse &response)
{
    response.remote_host = ros_node::m_driver->p_remote_host();

    return true;
}
bool ros_node::service_add_tcp_connection(driver_modem::AddTCPConnectionRequest& request, driver_modem::AddTCPConnectionResponse& response)
{
    response.success = ros_node::add_tcp_connection(static_cast<tcp_role>(request.role), request.port);

    return true;
}
bool ros_node::service_add_udp_connection(driver_modem::AddUDPConnectionRequest& request, driver_modem::AddUDPConnectionResponse& response)
{
    response.success = ros_node::add_udp_connection(request.port);

    return true;
}
bool ros_node::service_remove_connection(driver_modem::RemoveConnectionRequest& request, driver_modem::RemoveConnectionResponse& response)
{
    response.success = ros_node::remove_connection(static_cast<protocol>(request.protocol), request.port);

    return true;
}
bool ros_node::service_tcp_tx(driver_modem::SendTCPRequest &request, driver_modem::SendTCPResponse &response, uint16_t port)
{
    response.success = ros_node::m_driver->tx(protocol::TCP, port, request.packet.data.data(), static_cast<uint32_t>(request.packet.data.size()));

    return true;
}
