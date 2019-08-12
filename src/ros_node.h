/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

#include <driver_modem/DataPacket.h>
#include <driver_modem/SetRemoteHost.h>
#include <driver_modem/GetRemoteHost.h>
#include <driver_modem/AddTCPConnection.h>
#include <driver_modem/AddUDPConnection.h>
#include <driver_modem/RemoveConnection.h>
#include <driver_modem/RemoveAllConnections.h>
#include <driver_modem/SendTCP.h>

///
/// \brief Implements the driver's ROS node functionality.
///
class ros_node
{
public:
    // CONSTRUCTORS
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(int argc, char **argv);
    ~ros_node();

    // METHODS
    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    // VARIABLES
    ///
    /// \brief m_driver The driver instance.
    ///
    driver* m_driver;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;

    // VARIABLES: PUBLISHERS
    ///
    /// \brief m_publisher_active_connections The publisher for ActiveConnection messages.
    ///
    ros::Publisher m_publisher_active_connections;
    ///
    /// \brief m_tcp_rx The map of TCP RX publishers.
    ///
    std::map<uint16_t, ros::Publisher> m_tcp_rx;
    ///
    /// \brief m_udp_rx The map of UDP RX publishers.
    ///
    std::map<uint16_t, ros::Publisher> m_udp_rx;

    // VARIABLES: SUBSCRIBERS
    ///
    /// \brief m_udp_tx The map of UDP TX subscribers.
    ///
    std::map<uint16_t, ros::Subscriber> m_udp_tx;
    ///
    /// \brief m_tcp_tx The map of TCP TX service servers.
    ///
    std::map<uint16_t, ros::ServiceServer> m_tcp_tx;

    // VARIABLES: SERVICES
    ///
    /// \brief m_service_set_remote_host Service for setting the driver's remote host.
    ///
    ros::ServiceServer m_service_set_remote_host;
    ///
    /// \brief m_service_get_remote_host Service for geting the driver's remote host.
    ///
    ros::ServiceServer m_service_get_remote_host;
    ///
    /// \brief m_service_add_tcp_connection Service for adding TCP connections.
    ///
    ros::ServiceServer m_service_add_tcp_connection;
    ///
    /// \brief m_service_add_udp_connection Service for adding UDP connections.
    ///
    ros::ServiceServer m_service_add_udp_connection;
    ///
    /// \brief m_service_remove_connection Service for removing TCP/UDP connections.
    ///
    ros::ServiceServer m_service_remove_connection;
    ///
    /// \brief m_service_remove_all_connections Service for removing all connections.
    ///
    ros::ServiceServer m_service_remove_all_connections;

    // METHODS: CONNECTION MANAGEMENT
    ///
    /// \brief add_tcp_connection Instructs the driver to add a new TCP connection.
    /// \param role The role of the new TCP connection.
    /// \param port The port of the new TCP connection.
    /// \param publish_connections Indicates if the method should publish the ActiveConnections method.
    /// \return TRUE if the new connection was added, otherwise FALSE.
    ///
    bool add_tcp_connection(tcp_role role, uint16_t port, bool publish_connections = true);
    ///
    /// \brief add_udp_connection Instructs the driver to add a new UDP connection.
    /// \param port The port of the new UDP connection.
    /// \param publish_connections Indicates if the method should publish the ActiveConnections method.
    /// \return TRUE if the new connection was added, otherwise FALSE.
    ///
    bool add_udp_connection(uint16_t port, bool publish_connections = true);
    ///
    /// \brief remove_connection Instructs the driver to remove a TCP or UDP connection
    /// \param type The protocol type of the connection to remove.
    /// \param port The port of the connection to remove.
    /// \param publish_connections Indicates if the method should publish the ActiveConnections method.
    /// \return TRUE if the connection was removed, otherwise FALSE.
    ///
    bool remove_connection(protocol type, uint16_t port, bool publish_connections = true);
    ///
    /// \brief remove_all_connections Instructs the driver to remove all connections.
    ///
    void remove_all_connections(bool publish_connections = true);

    // METHODS: TOPIC MANAGEMENT
    ///
    /// \brief add_connection_topics Sets up publishers, subscribers, and services for new connections.
    /// \param type The protocol type of connection added.
    /// \param port The port of the connection added.
    ///
    void add_connection_topics(protocol type, uint16_t port);
    ///
    /// \brief remove_connection_topics Removes publishers, subscribers, and services for closed connections.
    /// \param type The protocol type of connected removed.
    /// \param port The port of the connection removed.
    ///
    void remove_connection_topics(protocol type, uint16_t port);

    // METHODS: MISC
    ///
    /// \brief publish_active_connections Publishes active connections.
    ///
    void publish_active_connections();

    // CALLBACKS: DRIVER
    ///
    /// \brief callback_tcp_connected The callback for handling driver TCP connection events.
    /// \param port The port of the new TCP connection.
    ///
    void callback_tcp_connected(uint16_t port);
    ///
    /// \brief callback_tcp_disconnected The callback for handling driver TCP disconnection events.
    /// \param port The port of the closed TCP connection.
    ///
    void callback_tcp_disconnected(uint16_t port);
    ///
    /// \brief callback_rx Handles the RX of data for all connections.
    /// \param type The connection type that received the data.
    /// \param port The local port that received the data.
    /// \param data The data that was received.
    /// \param length The length of the data that was received.
    /// \param source The IP address of the data source.
    ///
    void callback_rx(protocol type, uint16_t port, uint8_t* data, uint32_t length, address source);

    // CALLBACKS: SUBSCRIBERS
    ///
    /// \brief callback_udp_tx Forwards received DataPacket messages from udp tx topics.
    /// \param message The message to forward.
    /// \param port The local port to forward the message to.
    ///
    void callback_udp_tx(const driver_modem::DataPacketConstPtr& message, uint16_t port);

    // CALLBACKS: SERVICES
    ///
    /// \brief service_set_remote_host Service callback for setting the driver's remote host.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_set_remote_host(driver_modem::SetRemoteHostRequest& request, driver_modem::SetRemoteHostResponse& response);
    ///
    /// \brief service_get_remote_host Service callback for getting the driver's remote host.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_get_remote_host(driver_modem::GetRemoteHostRequest& request, driver_modem::GetRemoteHostResponse& response);
    ///
    /// \brief service_add_tcp_connection Service callback for adding TCP connectins.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_add_tcp_connection(driver_modem::AddTCPConnectionRequest& request, driver_modem::AddTCPConnectionResponse& response);
    ///
    /// \brief service_add_udp_connection Service callback for adding UDP connections.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_add_udp_connection(driver_modem::AddUDPConnectionRequest& request, driver_modem::AddUDPConnectionResponse& response);
    ///
    /// \brief service_remove_connection Service callback for removing TCP/UDP connections.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_remove_connection(driver_modem::RemoveConnectionRequest& request, driver_modem::RemoveConnectionResponse& response);
    ///
    /// \brief service_remove_all_connections Service callback for removing all active and pending connections.
    /// \param request The service request.
    /// \param response The service response.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_remove_all_connections(driver_modem::RemoveAllConnectionsRequest& request, driver_modem::RemoveAllConnectionsResponse& response);
    ///
    /// \brief service_tcp_tx Service for transmitting data over a TCP connection.
    /// \param request The service request.
    /// \param response The service response.
    /// \param port The TCP port to communicate the data over.
    /// \return TRUE if the service succeeded, otherwise FALSE.
    ///
    bool service_tcp_tx(driver_modem::SendTCPRequest& request, driver_modem::SendTCPResponse& response, uint16_t port);
};

#endif // ROS_NODE_H

