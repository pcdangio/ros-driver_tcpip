/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

#include <driver_modem/DataPacket.h>
#include <driver_modem/AddTCPConnection.h>
#include <driver_modem/AddUDPConnection.h>
#include <driver_modem/RemoveConnection.h>
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
    ///
    /// \brief m_publisher_active_connections The publisher for ActiveConnection messages.
    ///
    ros::Publisher m_publisher_active_connections;

    ros::ServiceServer m_service_add_tcp_connection;
    ros::ServiceServer m_service_add_udp_connection;
    ros::ServiceServer m_service_remove_tcp_connection;
    ros::ServiceServer m_service_remove_udp_connection;
    ///
    /// \brief m_tcp_rx The map of TCP RX publishers.
    ///
    std::map<uint16_t, ros::Publisher> m_tcp_rx;
    ///
    /// \brief m_udp_rx The map of UDP RX publishers.
    ///
    std::map<uint16_t, ros::Publisher> m_udp_rx;
    ///
    /// \brief m_udp_tx The map of UDP TX subscribers.
    ///
    std::map<uint16_t, ros::Subscriber> m_udp_tx;
    ///
    /// \brief m_tcp_tx The map of TCP TX service servers.
    ///
    std::map<uint16_t, ros::ServiceServer> m_tcp_tx;

    // METHODS
    bool add_tcp_connection(tcp_connection::role role, uint16_t port);
    bool add_udp_connection(uint16_t port);
    bool remove_connection(connection_type type, uint16_t port);
    ///
    /// \brief publish_active_connections Publishes active connections.
    ///
    void publish_active_connections();

    // DRIVER CALLBACKS
    ///
    /// \brief callback_rx Handles the RX of data for all connections.
    /// \param type The connection type that received the data.
    /// \param port The local port that received the data.
    /// \param data The data that was received.
    /// \param length The length of the data that was received.
    ///
    void callback_rx(connection_type type, uint16_t port, uint8_t* data, uint32_t length);
    ///
    /// \brief callback_disconnected Handles disconnect events for active connections.
    /// \param type The connection type that was disconnected.
    /// \param port The local port that was disconnected.
    ///
    void callback_disconnected(connection_type type, uint16_t port);

    // MESSAGE CALLBACKS
    ///
    /// \brief callback_udp_tx Forwards received DataPacket messages from udp tx topics.
    /// \param message The message to forward.
    /// \param port The local port to forward the message to.
    ///
    void callback_udp_tx(const driver_modem::DataPacketConstPtr& message, uint16_t port);

    // SERVICE_CALLBACKS
    bool service_add_tcp_connection(driver_modem::AddTCPConnectionRequest& request, driver_modem::AddTCPConnectionResponse& response);
    bool service_add_udp_connection(driver_modem::AddUDPConnectionRequest& request, driver_modem::AddUDPConnectionResponse& response);
    bool service_remove_connection(driver_modem::RemoveConnectionRequest& request, driver_modem::RemoveConnectionResponse& response, connection_type type);
    bool service_tcp_tx(driver_modem::SendTCPRequest& request, driver_modem::SendTCPResponse& response, uint16_t port);
};

#endif // ROS_NODE_H

