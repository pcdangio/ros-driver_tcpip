/// \file modem_interface.h
/// \brief Defines the driver_modem::modem_interface class.
#ifndef MODEM_INTERFACE_H
#define MODEM_INTERFACE_H

#include "driver_modem/protocol.h"
#include "driver_modem/tcp_role.h"

#include <ros/ros.h>

#include <driver_modem_msgs/active_connections.h>
#include <driver_modem_msgs/data_packet.h>

/// \brief Namespace for driver_modem package.
namespace driver_modem {

/// \brief Provides management and control of a driver_modem ROS node.
class modem_interface
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new modem manager.
    /// \param modem_name The global ROS name of the driver_modem node to manage.
    /// \note The namespace must be in the form of "\xxx\yyy\node_name"
    modem_interface(std::string modem_name);
    ~modem_interface();

    // METHODS: Callback Management
    /// \brief Attaches a callback for handling received TCP messages.
    /// \param callback The callback function.
    void attach_callback_tcp_rx(std::function<void(uint16_t, const driver_modem_msgs::data_packetConstPtr&)> callback);
    /// \brief Detaches the current callback for handling received TCP messages.
    void detach_callback_tcp_rx();
    /// \brief Attaches a callback for handling received UDP messages.
    /// \param callback The callback function.
    void attach_callback_udp_rx(std::function<void (uint16_t, const driver_modem_msgs::data_packetConstPtr &)> callback);
    /// \brief Detaches the current callback for handling received UDP messages.
    void detach_callback_udp_rx();


    // METHODS: Configuration
    /// \brief Sets the remote host of the modem.
    /// \param remote_host The new remote host to set.
    /// \return TRUE if successful, otherwise FALSE.
    bool set_remote_host(std::string remote_host);
    /// \brief Gets the remote host of the modem.
    /// \param remote_host A string to store the returned remote host in.
    /// \return TRUE if successful, otherwise FALSE.
    bool get_remote_host(std::string& remote_host);
    /// \brief Adds a TCP connection to the modem.
    /// \param role The role of the TCP connection (client or server).
    /// \param port The port of the TCP connection.
    /// \return TRUE if successful, otherwise FALSE.
    /// \note ROS takes time to connect publishers, subscribers, and services.
    /// The connection will not be immediately available for use.
    bool add_tcp_connection(tcp_role role, uint16_t port);
    /// \brief Adds a UDP connection to the modem.
    /// \param port The port of the UDP connection.
    /// \return TRUE if successful, otherwise FALSE.
    /// \note ROS takes time to connect publishers, subscribers, and services.
    /// The connection will not be immediately available for use.
    bool add_udp_connection(uint16_t port);
    /// \brief Removes a connection from the modem.
    /// \param type The protocol type of the connection to remove (TCP or UDP)
    /// \param port The port of the connection to remove.
    /// \return TRUE if successful, otherwise FALSE.
    bool remove_connection(protocol type, uint16_t port);
    /// \brief Removes all active and pending connections from the modem.
    /// \return TRUE if successful, otherwise FALSE.
    bool remove_all_connections();

    // METHODS: Transmission
    /// \brief Sends data via a TCP connection.
    /// \param port The port to send the data over.
    /// \param data The array of data to send.
    /// \param length The length of the data to send.
    /// \return TRUE if the message was sent, otherwise FALSE.
    /// FALSE can be returned if the TCP connection does not exist yet,
    /// or if the TCP transmission failed.
    bool send_tcp(uint16_t port, const uint8_t* data, uint32_t length);
    /// \brief Sends data via a UDP connection.
    /// \param port The port to send data over.
    /// \param data The array of data to send.
    /// \param length The length of the data to send.
    /// \return TRUE if the messsage was sent.  FALSE if the UDP connection does not exist yet.
    bool send_udp(uint16_t port, const uint8_t* data, uint32_t length);

    // METHODS: Connection Checking
    /// \brief Waits for the modem ROS node to become available.
    /// \param timeout The amount of time to wait.
    /// \return TRUE if the modem is available, FALSE if the wait times out.
    bool wait_for_modem(ros::Duration timeout = ros::Duration(-1));
    /// \brief Checks if a particular port is connected.
    /// \param type The protocol type of the connection.
    /// \param port The port to check.
    /// \return TRUE if the port is connected, FALSE if the port is pending or disconnected.
    bool is_connected(protocol type, uint16_t port) const;
    /// \brief Check is a TCP connection is pending.
    /// \param port The TCP port to check.
    /// \return TRUE if the port is pending a connection, otherwise false.
    bool is_pending(uint16_t port) const;
    /// \brief Waits until a connection is established.
    /// \param type THe protocol type of the connection.
    /// \param port The port of the connection.
    /// \param timeout Timeout (seconds) to wait for the connection.
    /// \return TRUE if the connection was found in time, otherwise FALSE.
    /// \details Blocks until connection is found, timeout, or the node shuts down.
    /// Spins the node while waiting.
    bool wait_for_connection(protocol type, uint16_t port, double_t timeout = 0.5) const;

    // PROPERTIES
    /// \brief Gets the active TCP connections.
    /// \return The list of active TCP ports.
    std::vector<uint16_t> p_active_tcp_connections() const;
    /// \brief Gets the pending TCP connections.
    /// \return The list of pending TCP ports.
    std::vector<uint16_t> p_pending_tcp_connections() const;
    /// \brief Gets the active UDP connections.
    /// \return The list of active UDP ports.
    std::vector<uint16_t> p_active_udp_connections() const;

private:
    // VARIABLES: Active Connections
    /// \brief List of active TCP connections.
    std::vector<uint16_t> m_active_tcp_connections;
    /// \brief List of pending TCP connections.
    std::vector<uint16_t> m_pending_tcp_connections;
    /// \brief List of active UDP connections.
    std::vector<uint16_t> m_active_udp_connections;

    // VARIABLES: External Callbacks
    /// \brief Stores the external callback for handling received TCP messages.
    std::function<void(uint16_t port, const driver_modem_msgs::data_packetConstPtr&)> m_callback_tcp_rx;
    /// \brief Stores the external callback for handling received UDP messages.
    std::function<void(uint16_t port, const driver_modem_msgs::data_packetConstPtr&)> m_callback_udp_rx;

    // VARIABLES: ROS Node
    /// \brief Maintains a copy of the ROS nodehandle for pub/sub/srv creation.
    ros::NodeHandle* m_node;

    // VARIABLES: Active Connections Subscriber
    /// \brief Subscriber for active_connections messages.
    ros::Subscriber m_subscriber_active_connections;

    // VARIABLES: Connection Management Service Clients
    /// \brief The SetRemoteHost service client.
    ros::ServiceClient m_service_set_remote_host;
    /// \brief The GetRemoteHost service client.
    ros::ServiceClient m_service_get_remote_host;
    /// \brief The AddTCPConnection service client.
    ros::ServiceClient m_service_add_tcp_connection;
    /// \brief The AddUDPConnection service client.
    ros::ServiceClient m_service_add_udp_connection;
    /// \brief The RemoveConnection service client.
    ros::ServiceClient m_service_remove_connection;

    // VARIABLES: Transmission Service Clients and Publishers
    /// \brief Service clients for sending TCP messages.
    std::map<uint16_t, ros::ServiceClient> m_services_send_tcp;
    /// \brief Publishers for sending UDP messages.
    std::map<uint16_t, ros::Publisher> m_publishers_udp;
    /// \brief Subscribers for TCP RX messages.
    std::map<uint16_t, ros::Subscriber> m_subscribers_tcp_rx;
    /// \brief Subscribers for UDP RX messages.
    std::map<uint16_t, ros::Subscriber> m_subscribers_udp_rx;

    // METHODS
    /// \brief Removes duplicate ports from two lists.
    /// \param a The first list of ports.
    /// \param b The second list of ports.
    void remove_duplicates(std::list<uint16_t>& a, std::list<uint16_t>& b);

    // CALLBACKS: Subscribers
    /// \brief Handles active_connections messages.
    /// \param message
    void callback_active_connections(const driver_modem_msgs::active_connectionsPtr& message);
    /// \brief Handles received TCP messages.
    /// \param message The recieved message.
    /// \param port The port the message was received on.
    void callback_tcp_rx(const driver_modem_msgs::data_packetConstPtr &message, uint16_t port);
    /// \brief Handles received UDP messages.
    /// \param message The received message.
    /// \param port The port the message was received on.
    void callback_udp_rx(const driver_modem_msgs::data_packetConstPtr& message, uint16_t port);
};

}

#endif // MODEM_INTERFACE_H
