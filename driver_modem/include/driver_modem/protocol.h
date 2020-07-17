/// \file protocol.h
/// \brief Defines the protocol enumeration.
#ifndef PROTOCOL_H
#define PROTOCOL_H

///
/// \brief Namespace for driver_modem package.
///
namespace driver_modem {

///
/// \brief An enumeration of TCP/IP connection protocol types.
///
enum class protocol
{
    TCP = 0,    ///< TCP Protocol
    UDP = 1     ///< UDP Protocol
};

}

#endif // PROTOCOL_H
