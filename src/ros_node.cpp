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
    ros_node::m_node->param<std::string>("local_ip", param_local_ip, "192.168.1.243");
    std::string param_remote_ip;
    ros_node::m_node->param<std::string>("remote_ip", param_remote_ip, "192.168.1.243");

    // Read connect port parameters.
    std::vector<int> param_tcp_local_ports = {4000, 4002, 4004};
    //ros_node::m_node->getParam("tcp_local_ports", param_tcp_local_ports);
    std::vector<int> param_tcp_remote_ports = {4001, 4003, 4005};
    //ros_node::m_node->getParam("tcp_remote_ports", param_tcp_remote_ports);
    std::vector<int> param_udp_local_ports = {3000, 3002, 3004};
    //ros_node::m_node->getParam("udp_local_ports", param_udp_local_ports);
    std::vector<int> param_udp_remote_ports = {3001, 3003, 3005};
    //ros_node::m_node->getParam("udp_remote_ports", param_udp_remote_ports);

    // Initialize driver.
    ros_node::m_driver = new driver(param_local_ip,
                                    param_remote_ip,
                                    std::bind(&ros_node::rx_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
                                    std::bind(&ros_node::disconnect_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Set up active connections publisher.
    // This will publish each time the connections are modified.
    // Use latching so new nodes always have the latest information.
    ros_node::m_publisher_active_connections = ros_node::m_node->advertise<driver_modem::ActiveConnections>("active_connections", 1, true);

    // Set up service server for modifying connections.
    ros_node::m_service_modify_connection = ros_node::m_node->advertiseService("modify_connection", &ros_node::modify_connection, this);

    // Set up tx/rx publishers, subscribers, and services.

    // TCP:
    // Verify that equal amount of local and remote ports were provided as parameters.
    if(param_tcp_local_ports.size() == param_tcp_remote_ports.size())
    {
        for(uint32_t i = 0; i < param_tcp_local_ports.size(); i++)
        {
            // Get local/remote port from vectors.
            uint16_t local_port = static_cast<uint16_t>(param_tcp_local_ports.at(i));
            uint16_t remote_port = static_cast<uint16_t>(param_tcp_remote_ports.at(i));

            // Add connection to driver.
            if(ros_node::m_driver->add_connection(connection_type::TCP, local_port, remote_port))
            {
                // RX Publisher:
                // Generate topic name.
                std::stringstream rx_topic;
                rx_topic << "tcp/" << local_port << "/rx";
                // Add new rx publisher to the map.
                ros_node::m_tcp_rx.insert(std::make_pair(local_port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

                // TX Service:
                // Generate topic name.
                std::stringstream tx_topic;
                tx_topic << "tcp/" << local_port << "/tx";
                // Add new tx service server to the map.
                ros_node::m_tcp_tx.insert(std::make_pair(local_port, ros_node::m_node->advertiseService<driver_modem::TCPtxRequest, driver_modem::TCPtxResponse>(tx_topic.str(), std::bind(&ros_node::tcp_tx, this, std::placeholders::_1, std::placeholders::_2, local_port))));
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Invalid parameters: Number of tcp_local_ports differs from number of tcp_remote_ports.");
    }

    // UDP:
    if(param_udp_local_ports.size() == param_udp_remote_ports.size())
    {
        for(uint32_t i = 0; i < param_udp_local_ports.size(); i++)
        {
            // Get local/remote ports from vectors.
            uint16_t local_port = static_cast<uint16_t>(param_udp_local_ports.at(i));
            uint16_t remote_port = static_cast<uint16_t>(param_udp_remote_ports.at(i));

            // Add connection to driver.
            if(ros_node::m_driver->add_connection(connection_type::UDP, local_port, remote_port))
            {
                // RX Publisher:
                // Generate topic name.
                std::stringstream rx_topic;
                rx_topic << "udp/" << local_port << "/rx";
                // Add new rx publisher to the map.
                ros_node::m_udp_rx.insert(std::make_pair(local_port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

                // TX Subscriber:
                // Generate topic name.
                std::stringstream tx_topic;
                tx_topic << "udp/" << local_port << "/tx";
                // Add new tx subscriber to the map.
                ros_node::m_udp_tx.insert(std::make_pair(local_port, ros_node::m_node->subscribe<driver_modem::DataPacket>(tx_topic.str(), 1, std::bind(&ros_node::udp_tx, this, std::placeholders::_1, local_port))));
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Invalid parameters: Number of udp_local_ports differs from number of udp_remote_ports.");
    }

    // Publish active connections now that all are initialized.
    ros_node::publish_active_connections();

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
void ros_node::publish_active_connections()
{
    // Convert current connections into ActiveConnections message.

    // Create output message.
    driver_modem::ActiveConnections message;
    message.header.stamp = ros::Time::now();

    // Get the list of active connections from the driver.
    std::vector<std::pair<connection_type, uint16_t>> connections = ros_node::m_driver->p_connections();

    // Iterate over active connections and add to message.
    for(uint32_t i = 0; i < connections.size(); i++)
    {
        std::pair<connection_type, uint16_t>& current = connections.at(i);

        switch(current.first)
        {
        case connection_type::TCP:
        {
            message.tcp.push_back(current.second);
            break;
        }
        case connection_type::UDP:
        {
            message.udp.push_back(current.second);
            break;
        }
        }
    }

    // Publish the message.
    ros_node::m_publisher_active_connections.publish(message);
}

// MESSAGE CALLBACKS
void ros_node::udp_tx(const driver_modem::DataPacketConstPtr &message, uint16_t local_port)
{
    ros_node::m_driver->tx(connection_type::UDP, local_port, message->data.data(), static_cast<uint32_t>(message->data.size()));
}
bool ros_node::tcp_tx(driver_modem::TCPtxRequest &request, driver_modem::TCPtxResponse &response, uint16_t local_port)
{
    bool result = ros_node::m_driver->tx(connection_type::TCP, local_port, request.packet.data.data(), static_cast<uint32_t>(request.packet.data.size()));
    response.success = result;
    return result;
}
bool ros_node::modify_connection(driver_modem::ModifyConnectionRequest &request, driver_modem::ModifyConnectionResponse &response)
{
    bool result = false;

    // Try to fulfill the request.
    if(request.add)
    {
        // Adding a connection.
        result = ros_node::m_driver->add_connection(static_cast<connection_type>(request.connection_type), request.local_port, request.remote_port);
    }
    else
    {
        // Removing a connection.
        result = ros_node::m_driver->remove_connection(static_cast<connection_type>(request.connection_type), request.local_port);
    }

    // If the request succeeded, a connection change was made.  Republish active connections.
    if(result)
    {
        ros_node::publish_active_connections();
    }

    response.success = result;

    return result;
}

// CALLBACKS
void ros_node::rx_callback(connection_type type, uint16_t local_port, uint8_t *data, uint32_t length)
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
        ros_node::m_tcp_rx.at(local_port).publish(message);
        break;
    }
    case connection_type::UDP:
    {
        ros_node::m_udp_rx.at(local_port).publish(message);
        break;
    }
    }
}
void ros_node::disconnect_callback(connection_type type, uint16_t local_port)
{
    // Remove the associated topic/service.
    switch(type)
    {
    case connection_type::TCP:
    {
        ros_node::m_tcp_rx.at(local_port).shutdown();
        ros_node::m_tcp_rx.erase(local_port);
        ros_node::m_tcp_tx.at(local_port).shutdown();
        ros_node::m_tcp_tx.erase(local_port);
        break;
    }
    case connection_type::UDP:
    {
        ros_node::m_tcp_rx.at(local_port).shutdown();
        ros_node::m_tcp_rx.erase(local_port);
        ros_node::m_tcp_tx.at(local_port).shutdown();
        ros_node::m_tcp_tx.erase(local_port);
        break;
    }
    }

    // Publish updated connections.
    ros_node::publish_active_connections();
}
