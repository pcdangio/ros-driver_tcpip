/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include "tcp_connection.h"
#include "udp_connection.h"

#include <boost/thread.hpp>

///
/// \brief A driver for TCP/UDP communications over a network interface.
///
class driver
{
public:
    // CONSTRUCTORS

    driver(std::string local_ip, std::string remote_ip,
           std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> rx_callback,
           std::function<void(uint16_t)> tcp_connected_callback,
           std::function<void(uint16_t)> tcp_disconnected_callback);
    ~driver();

    // METHODS
    ///
    /// \brief start Starts the IO service event loop in a separate thread.
    ///
    void start();
    ///
    /// \brief stop Stops the IO service event loop running in the separate thread.
    ///
    void stop();


    bool add_udp_connection(uint16_t local_port, uint16_t remote_port);
    bool add_tcp_connection(tcp_connection::role role, uint16_t port);
    ///
    /// \brief remove_connection Removes an existing TCP/UDP connection.
    /// \param type The type of connection to remove (TCP or UDP).
    /// \param port The port of the connection.
    /// \return TRUE if the connection was removed, otherwise FALSE.
    ///
    bool remove_connection(connection_type type, uint16_t port);
    ///
    /// \brief tx Transmits data over a connection.
    /// \param type The connection type to transmit via.
    /// \param port The port to transmit from.
    /// \param data The array of data to transmit.
    /// \param length The length of the data to transmit.
    /// \return TRUE if the transmit operation succeeded, otherwise FALSE.
    /// \note This method takes ownership of the data pointer.
    ///
    bool tx(connection_type type, uint16_t port, const uint8_t* data, uint32_t length);

    // PROPERTIES
    std::vector<uint16_t> p_pending_tcp_connections() const;
    std::vector<uint16_t> p_active_tcp_connections() const;
    std::vector<uint16_t> p_active_udp_connections() const;

private:
    // VARIABLES
    ///
    /// \brief m_service Stores the driver's io_service instance.
    ///
    boost::asio::io_service m_service;
    ///
    /// \brief m_thread A separate thread for running the IO service event loop.
    ///
    boost::thread m_thread;

    std::map<uint16_t, tcp_connection*> m_tcp_active;
    std::map<uint16_t, tcp_connection*> m_tcp_pending;
    std::map<uint16_t, udp_connection*> m_udp_active;
    ///
    /// \brief m_local_ip Stores the local IP address for all connections.
    ///
    boost::asio::ip::address m_local_ip;
    ///
    /// \brief m_remote_ip Stores the remote IP address for all connections.
    ///
    boost::asio::ip::address m_remote_ip;

    std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> m_callback_rx;
    std::function<void(uint16_t)> m_callback_tcp_connected;
    std::function<void(uint16_t)> m_callback_tcp_disconnected;

    // CALLBACKS
    void callback_tcp_connected(uint16_t port);
    void callback_tcp_disconnected(uint16_t port);
};

#endif // DRIVER_H
