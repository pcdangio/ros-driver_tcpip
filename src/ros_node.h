/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

#include <driver_modem/DataPacket.h>
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
    std::map<uint16_t, ros::Publisher> m_tcp_rx;
    std::map<uint16_t, ros::Publisher> m_udp_rx;
    std::map<uint16_t, ros::Subscriber> m_udp_tx;
    std::map<uint16_t, ros::ServiceServer> m_tcp_tx;

    ros::Rate* m_spin_rate;

    // CALLBACKS
    void rx_callback(connection_type type, uint16_t local_port, uint8_t* data, uint32_t length);
    void disconnect_callback(connection_type type, uint16_t local_port);

    // MESSAGE CALLBACKS
    void udp_tx(const driver_modem::DataPacketPtr& message);
    bool tcp_tx(driver_modem::TCPtxRequest& request, driver_modem::TCPtxResponse& response);
};

#endif // ROS_NODE_H

