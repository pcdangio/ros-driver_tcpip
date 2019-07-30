/// \file connection_type.h
/// \brief Defines the connection_type enumeration.
#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

///
/// \brief An enumeration of TCP/IP connection protocol types.
///
enum class connection_type
{
    TCP = 0,    ///< TCP Protocol
    UDP = 1     ///< UDP Protocol
};

#endif // CONNECTION_TYPE_H
