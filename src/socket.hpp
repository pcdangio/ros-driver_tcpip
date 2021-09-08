/// \file socket.hpp
/// \brief Defines the driver_tcpip::socket_t class.
#ifndef DRIVER_TCPIP___SOCKET_H
#define DRIVER_TCPIP___SOCKET_H

#include "protocol.hpp"

#include <stdint.h>

namespace driver_tcpip {

/// \brief An abstract base class for IP sockets.
class socket_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new socket_t.
    /// \param id The unique ID of the socket.
    /// \param protocol The IP protocol of the socket.
    socket_t(uint32_t id, protocol_t protocol);

    // METHODS
    /// \brief Closes the socket.
    virtual void close() = 0;

    // PROPERTIES
    /// \brief Indicates the IP protocol of the socket.
    /// \returns The IP protocol of the socket.
    protocol_t protocol() const;
    /// \brief Indicates if the socket is open.
    /// \returns TRUE if the socket is open, otherwise FALSE.
    virtual bool is_open() const = 0;

protected:
    // VARIABLES
    /// \brief The unique ID of the socket.
    const uint32_t m_id;
    /// \brief The IP protocol of the socket.
    const protocol_t m_protocol;
};

}

#endif