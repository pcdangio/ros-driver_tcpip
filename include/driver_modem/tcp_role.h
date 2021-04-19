/// \file tcp_role.h
/// \brief Defines the tcp_role enumeration.
#ifndef TCP_ROLE_H
#define TCP_ROLE_H

/// \brief Namespace for driver_modem package.
namespace driver_modem {

/// \brief Enumerates the roles that a TCP connection may take.
enum class tcp_role
{
    UNASSIGNED = 0,     ///< The connection has not yet been assigned a role
    SERVER = 1,         ///< The connection is acting as a TCP server
    CLIENT = 2          ///< The connection is acting as a TCP client
};

}

#endif // TCP_ROLE_H
