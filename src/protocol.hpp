/// \file protocol.hpp
/// \brief Defines the driver_modem::protocol_t enumeration.
#ifndef DRIVER_MODEM___PROTOCOL_H
#define DRIVER_MODEM___PROTOCOL_H

namespace driver_modem {

/// \brief Specifies network protocol types.
enum class protocol_t
{
    UDP = 0,    ///< User Datagram Protocol (UDP)
    TCP = 1     ///< Transmission Control Protocol (TCP)
};

}

#endif