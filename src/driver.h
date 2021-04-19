/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include "tcp_connection.h"
#include "udp_connection.h"

#include <boost/thread.hpp>

/// \brief A driver for TCP/UDP communications over a network interface.
class driver
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    /// \param local_ip The local IP address of that the connection shall bind to.
    /// \param remote_host The remote IP or hostname to communicate with.
    /// \param rx_callback A callback for handling received TCP/UDP messages.
    /// \param tcp_connected_callback A callback for handling TCP connection events.
    /// \param tcp_disconnected_callback A callback for handling TCP disconnection events.
    driver(std::string local_ip, std::string remote_host,
           std::function<void(protocol, uint16_t, uint8_t*, uint32_t, address)> rx_callback,
           std::function<void(uint16_t)> tcp_connected_callback,
           std::function<void(uint16_t)> tcp_disconnected_callback);
    ~driver();

    // METHODS: START/STOP
    /// \brief Starts the IO service event loop in a separate thread.
    void start();
    /// \brief Stops the IO service event loop running in the separate thread.
    void stop();

    // METHODS: CONNECTION MANAGEMENT
    /// \brief Sets the remote host of the driver.
    /// \param remote_host The remote host to communicate with.
    /// \return TRUE if the remove host could be resolved, otheriwse FALSE.
    bool set_remote_host(std::string remote_host);
    /// \brief Adds a TCP connection to the driver.
    /// \param role The role that the TCP connection should operate as.
    /// \param port The port that the connection shall communicate through.
    /// \return TRUE if the connection was added, otherwise FALSE.
    bool add_tcp_connection(tcp_role role, uint16_t port);
    /// \brief Adds a UDP connection to the driver.
    /// \param port The port that the connection shall communicate through.
    /// \return TRUE if the connection was added, otherwise FALSE.
    bool add_udp_connection(uint16_t port);
    /// \brief Removes an existing TCP/UDP connection.
    /// \param type The protocol type of connection to remove (TCP or UDP).
    /// \param port The port of the connection.
    /// \return TRUE if the connection was removed, otherwise FALSE.
    bool remove_connection(protocol type, uint16_t port);
    /// \brief Removes all active and pending connections.
    void remove_all_connections();

    // METHODS: IO
    /// \brief Transmits data over a connection.
    /// \param type The connection type to transmit via.
    /// \param port The port to transmit from.
    /// \param data The array of data to transmit.
    /// \param length The length of the data to transmit.
    /// \return TRUE if the transmit operation succeeded, otherwise FALSE.
    /// \note This method takes ownership of the data pointer.
    bool tx(protocol type, uint16_t port, const uint8_t* data, uint32_t length);

    // METHODS: Static
    /// \brief Gets the string representation of a protocol.
    /// \param value The protocol value.
    /// \return The string representation of the protocol.
    static std::string protocol_string(protocol value);
    /// \brief Gets the string representation of a tcp_role.
    /// \param value The tcp_role value.
    /// \return The string representation of the tcp_role.
    static std::string tcp_role_string(tcp_role value);

    // PROPERTIES
    /// \brief Gets the current remote host of the driver.
    /// \return The remote host of the driver.
    std::string p_remote_host();
    /// \brief Gets the list of pending TCP connections.
    /// \return The list of pending TCP connections.
    std::vector<uint16_t> p_pending_tcp_connections() const;
    /// \brief Gets the list of active TCP connections.
    /// \return The list of active TCP connections.
    std::vector<uint16_t> p_active_tcp_connections() const;
    /// \brief Gets the list of active UDP connections.
    /// \return The list of active UDP connections.
    std::vector<uint16_t> p_active_udp_connections() const;

private:
    // VARIABLES: SOCKET
    /// \brief Stores the driver's io_service instance.
    boost::asio::io_service m_service;
    /// \brief Stores the local IP address for all connections.
    boost::asio::ip::address m_local_ip;
    /// \brief Stores the remote IP address for all connections.
    boost::asio::ip::address m_remote_ip;
    /// \brief A separate thread for running the IO service event loop.
    boost::thread m_thread;
    /// \brief IO service worker instance for keepign io_service::run running.
    boost::asio::io_service::work* m_service_work;

    // VARIABLES: CONNECTION MAPS
    /// \brief The map of pending TCP connections.
    std::map<uint16_t, boost::shared_ptr<tcp_connection>> m_tcp_pending;
    /// \brief The map of active TCP connections.
    std::map<uint16_t, boost::shared_ptr<tcp_connection>> m_tcp_active;
    /// \brief The map of active UDP connections.
    std::map<uint16_t, boost::shared_ptr<udp_connection>> m_udp_active;

    // VARIABLES: CALLBACKS
    /// \brief The callback to raise when TCP connections are made.
    std::function<void(uint16_t)> m_callback_tcp_connected;
    /// \brief The callback to raise when TCP disconnections occur.
    std::function<void(uint16_t)> m_callback_tcp_disconnected;
    /// \brief The callback to raise when messages are received.
    std::function<void(protocol, uint16_t, uint8_t*, uint32_t, address)> m_callback_rx;

    // CALLBACKS
    /// \brief The callback for handling TCP connection events.
    /// \param port The port of the connection that was connected.
    void callback_tcp_connected(uint16_t port);
    /// \brief The callback for handling TCP disconnection events.
    /// \param port The port of the connection that was disconnected.
    void callback_tcp_disconnected(uint16_t port);
};

#endif // DRIVER_H
