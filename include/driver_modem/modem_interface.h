/// \file modem_interface.h
/// \brief Defines the modem_interface class.
#ifndef MODEM_INTERFACE_H
#define MODEM_INTERFACE_H

#include <ros/ros.h>

#include <driver_modem/ActiveConnections.h>
#include <driver_modem/DataPacket.h>

///
/// \brief Namespace for driver_modem package.
///
namespace driver_modem {

///
/// \brief Provides management and control of a driver_modem ROS node.
///
class modem_interface
{
public:
    // ENUMERATIONS
    ///
    /// \brief An enumeration of communication protocols.
    ///
    enum class protocol
    {
        TCP = 0,    ///< TCP Protocol
        UDP = 1     ///< UDP Protocol
    };
    ///
    /// \brief An enumeration of TCP roles.
    ///
    enum class tcp_role
    {
        SERVER = 0,     ///< TCP Server Connection
        CLIENT = 1      ///< TCP Client Connection
    };

    // CONSTRUCTORS
    ///
    /// \brief modem_interface Creates a new modem manager.
    /// \param modem_namespace The global namespace of the driver_modem node to manage.
    /// \note The namespace must be in the form of "\xxx\yyy\node_name"
    ///
    modem_interface(std::string modem_namespace);
    ~modem_interface();

    // METHODS: Callback Management
    ///
    /// \brief attach_callback_tcp_rx Attaches a callback for handling received TCP messages.
    /// \param callback The callback function.
    ///
    void attach_callback_tcp_rx(std::function<void(const driver_modem::DataPacketPtr&)> callback);
    ///
    /// \brief detach_callback_tcp_rx Detaches the current callback for handling received TCP messages.
    ///
    void detach_callback_tcp_rx();
    ///
    /// \brief attach_callback_udp_rx Attaches a callback for handling received UDP messages.
    /// \param callback The callback function.
    ///
    void attach_callback_udp_rx(std::function<void(const driver_modem::DataPacketPtr&)> callback);
    ///
    /// \brief detach_callback_udp_rx Detaches the current callback for handling received UDP messages.
    ///
    void detach_callback_udp_rx();


    // METHODS: Configuration
    ///
    /// \brief set_remote_host Sets the remote host of the modem.
    /// \param remote_host The new remote host to set.
    /// \return TRUE if successful, otherwise FALSE.
    ///
    bool set_remote_host(std::string remote_host);
    ///
    /// \brief get_remote_host Gets the remote host of the modem.
    /// \param remote_host A string to store the returned remote host in.
    /// \return TRUE if successful, otherwise FALSE.
    ///
    bool get_remote_host(std::string& remote_host);
    ///
    /// \brief add_tcp_connection Adds a TCP connection to the modem.
    /// \param role The role of the TCP connection (client or server).
    /// \param port The port of the TCP connection.
    /// \return TRUE if successful, otherwise FALSE.
    ///
    bool add_tcp_connection(tcp_role role, uint16_t port);
    ///
    /// \brief add_udp_connection Adds a UDP connection to the modem.
    /// \param port The port of the UDP connection.
    /// \return TRUE if successful, otherwise FALSE.
    ///
    bool add_udp_connection(uint16_t port);
    ///
    /// \brief remove_connection Removes a connection from the modem.
    /// \param type The protocol type of the connection to remove (TCP or UDP)
    /// \param port The port of the connection to remove.
    /// \return TRUE if successful, otherwise FALSE.
    ///
    bool remove_connection(protocol type, uint16_t port);

    // METHODS: Transmission
    ///
    /// \brief send_tcp Sends data via a TCP connection.
    /// \param port The port to send the data over.
    /// \param data The array of data to send.
    /// \param length The length of the data to send.
    /// \return TRUE if the message was sent, otherwise FALSE.
    /// FALSE can be returned if the TCP connection does not exist yet,
    /// or if the TCP transmission failed.
    ///
    bool send_tcp(uint16_t port, const uint8_t* data, uint32_t length);
    ///
    /// \brief send_udp Sends data via a UDP connection.
    /// \param port The port to send data over.
    /// \param data The array of data to send.
    /// \param length The length of the data to send.
    /// \return TRUE if the messsage was sent.  FALSE if the UDP connection does not exist yet.
    ///
    bool send_udp(uint16_t port, const uint8_t* data, uint32_t length);

private:
    // VARIABLES: Active Connections
    ///
    /// \brief m_active_tcp_connections List of active TCP connections.
    ///
    std::vector<uint16_t> m_active_tcp_connections;
    ///
    /// \brief m_pending_tcp_connections List of pending TCP connections.
    ///
    std::vector<uint16_t> m_pending_tcp_connections;
    ///
    /// \brief m_active_udp_connections List of active UDP connections.
    ///
    std::vector<uint16_t> m_active_udp_connections;

    // VARIABLES: External Callbacks
    ///
    /// \brief m_callback_tcp_rx Stores the external callback for handling received TCP messages.
    ///
    std::function<void(const driver_modem::DataPacketPtr&)> m_callback_tcp_rx;
    ///
    /// \brief m_callback_udp_rx Stores the external callback for handling received UDP messages.
    ///
    std::function<void(const driver_modem::DataPacketPtr&)> m_callback_udp_rx;

    // VARIABLES: ROS Node
    ///
    /// \brief m_node Maintains a copy of the ROS nodehandle for pub/sub/srv creation.
    ///
    ros::NodeHandle* m_node;

    // VARIABLES: Active Connections Subscriber
    ///
    /// \brief m_subscriber_active_connections Subscriber for ActiveConnections messages.
    ///
    ros::Subscriber m_subscriber_active_connections;

    // VARIABLES: Connection Management Service Clients
    ///
    /// \brief m_service_set_remote_host The SetRemoteHost service client.
    ///
    ros::ServiceClient m_service_set_remote_host;
    ///
    /// \brief m_service_get_remote_host The GetRemoteHost service client.
    ///
    ros::ServiceClient m_service_get_remote_host;
    ///
    /// \brief m_service_add_tcp_connection The AddTCPConnection service client.
    ///
    ros::ServiceClient m_service_add_tcp_connection;
    ///
    /// \brief m_service_add_udp_connection The AddUDPConnection service client.
    ///
    ros::ServiceClient m_service_add_udp_connection;
    ///
    /// \brief m_service_remove_connection The RemoveConnection service client.
    ///
    ros::ServiceClient m_service_remove_connection;

    // VARIABLES: Transmission Service Clients and Publishers
    ///
    /// \brief m_services_send_tcp Service clients for sending TCP messages.
    ///
    std::map<uint16_t, ros::ServiceClient> m_services_send_tcp;
    ///
    /// \brief m_publishers_udp Publishers for sending UDP messages.
    ///
    std::map<uint16_t, ros::Publisher> m_publishers_udp;
    ///
    /// \brief m_subscribers_tcp_rx Subscribers for TCP RX messages.
    ///
    std::map<uint16_t, ros::Subscriber> m_subscribers_tcp_rx;
    ///
    /// \brief m_subscribers_udp_rx Subscribers for UDP RX messages.
    ///
    std::map<uint16_t, ros::Subscriber> m_subscribers_udp_rx;

    // METHODS
    ///
    /// \brief remove_duplicates Removes duplicate ports from two lists.
    /// \param a The first list of ports.
    /// \param b The second list of ports.
    ///
    void remove_duplicates(std::list<uint16_t>& a, std::list<uint16_t>& b);

    // CALLBACKS: Subscribers
    ///
    /// \brief callback_active_connections Handles ActiveConnections messages.
    /// \param message
    ///
    void callback_active_connections(const driver_modem::ActiveConnectionsPtr& message);
    ///
    /// \brief callback_tcp_rx Handles received TCP messages.
    /// \param message The recieved message.
    ///
    void callback_tcp_rx(const driver_modem::DataPacketPtr& message);
    ///
    /// \brief callback_udp_rx Handles received UDP messages.
    /// \param message The received message.
    ///
    void callback_udp_rx(const driver_modem::DataPacketPtr& message);
};

}

#endif // MODEM_INTERFACE_H
