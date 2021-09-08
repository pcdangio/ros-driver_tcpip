/// \file protocol.hpp
/// \brief Defines the driver_tcpip::protocol_t enumeration.
#ifndef DRIVER_TCPIP___PROTOCOL_H
#define DRIVER_TCPIP___PROTOCOL_H

namespace driver_tcpip {

/// \brief Specifies network protocol types.
enum class protocol_t
{
    UDP = 0,    ///< User Datagram Protocol (UDP)
    TCP = 1     ///< Transmission Control Protocol (TCP)
};

}

#endif