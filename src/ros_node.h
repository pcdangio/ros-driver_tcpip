/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

#include <driver_modem/DataPacket.h>
#include <driver_modem/ModifyConnection.h>
#include <driver_modem/TCPtx.h>

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
    ///
    /// \brief m_service_modify_connection The service server for modifying connections.
    ///
    ros::ServiceServer m_service_modify_connection;
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
    ///
    /// \brief publish_active_connections Publishes active connections.
    ///
    void publish_active_connections();

    // CALLBACKS
    ///
    /// \brief rx_callback Handles the RX of data for all connections.
    /// \param type The connection type that received the data.
    /// \param local_port The local port that received the data.
    /// \param data The data that was received.
    /// \param length The length of the data that was received.
    ///
    void rx_callback(connection_type type, uint16_t local_port, uint8_t* data, uint32_t length);
    ///
    /// \brief disconnect_callback Handles disconnect events for active connections.
    /// \param type The connection type that was disconnected.
    /// \param local_port The local port that was disconnected.
    ///
    void disconnect_callback(connection_type type, uint16_t local_port);

    // MESSAGE CALLBACKS
    ///
    /// \brief udp_tx Forwards received DataPacket messages from udp tx topics.
    /// \param message The message to forward.
    /// \param local_port The local port to forward the message to.
    ///
    void udp_tx(const driver_modem::DataPacketConstPtr& message, uint16_t local_port);
    ///
    /// \brief tcp_tx A service that forwards DataPacket messages from tcp tx services.
    /// \param request The service request containing the data to forward.
    /// \param response The response indicating if the service succeeded.
    /// \param local_port The local port to forward the data to.
    /// \return TRUE if the data was forwarded, otherwise FALSE.
    ///
    bool tcp_tx(driver_modem::TCPtxRequest& request, driver_modem::TCPtxResponse& response, uint16_t local_port);
    ///
    /// \brief modify_connection A service that enables connections to be modified.
    /// \param request The service request containing the modify command.
    /// \param response The response indicating if the service succeeded.
    /// \return TRUE if the connection was modified, otherwise FALSE.
    ///
    bool modify_connection(driver_modem::ModifyConnectionRequest& request, driver_modem::ModifyConnectionResponse& response);
};

#endif // ROS_NODE_H

