# driver_modem

## Overview

This package includes driver software for TCP/IP communications outside of ROS. It enables sending TCP and/or UDP packets via several ports over an existing network interface.

**Keywords:** modem driver tcp udp ip network communication

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_modem package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [std_msgs](http://wiki.ros.org/std_msgs) (ROS std_msgs)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_modem.git driver_modem
        cd ../
        catkin_make

## Usage

Run the driver with the following command:

        rosrun driver_modem node

## Nodes

### node

A driver for managing TCP/IP communications over a network interface.  This node will create topics for reading from and writing to TCP and/or UDP ports that are opened via the driver.
Ports can be opened via parameters or through the modify_connection service.  Any data that is published to a "tx" topic will be sent over the network interface.
Any data that is read from the network device will be published on the "rx" topics.


#### Published Topics
* **`~/active_connections`** ([driver_modem/ActiveConnections](https://github.com/pcdangio/ros-driver_modem/blob/master/msg/ActiveConnections.msg))

        Publishes (with latching) a list of all active connections managed by the driver each time a new connection is added or an existing connection is removed.

* **`~/PROTOCOL_TYPE/PORT/rx`** ([driver_modem/DataPacket](https://github.com/pcdangio/ros-driver_modem/blob/master/msg/DataPacket.msg))

        Publishes data that has been received over a particular protocol and port.
        PROTOCOL_TYPE: Either "tcp" or "udp" depending on the connection protocol
        PORT: The port number of the connection.

#### Subscribed Topics
* **`~/udp/PORT/tx`** ([driver_modem/DataPacket](https://github.com/pcdangio/ros-driver_modem/blob/master/msg/DataPacket.msg))

        Accepts data to send via UDP over a particular port.
        PORT: The port number of the connection.

#### Services
* **`~/set_remote_host`** ([driver_modem/SetRemoteHost](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/SetRemoteHost.srv))

        Sets the remote host that outgoing UDP and TCP connections will communicate with.
        *NOTE* This will close all existing connections.

* **`~/get_remote_host`** ([driver_modem/GetRemoteHost](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/GetRemoteHost.srv))

        Gets the remote host that outgoing UDP and TCP connections will communicate with.

* **`~/add_tcp_connection`** ([driver_modem/AddTCPConnection](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/AddTCPConnection.srv))

        Adds a new TCP connection to the driver.

* **`~/add_udp_connection`** ([driver_modem/AddUDPConnection](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/AddUDPConnection.srv))

        Adds a new UDP connection to the driver.

* **`~/remove_connection`** ([driver_modem/RemoveConnection](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/RemoveConnection.srv))

        Removes a TCP or UDP connection from the driver.

* **`~/tcp/PORT/tx`** ([driver_modem/SendTCP](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/SendTCP.srv))

        Accepts data to send via TCP over a particular port.  This is implemented as a service to indicate success.
        PORT: The port number of the connection.

#### Runtime Parameters

* **`~/local_ip`** (string, default: 192.168.1.2)

        The IP address of the local network interface to use for communication.

* **`~/remote_host`** (string, default: 192.168.1.3)

        The hostname or IP address of the remote device to communicate with.

#### Connection Parameters

These parameters are optional and can be used to create TCP and/or UDP connections on node startup.

* **`~/tcp_server_ports`** (vector<uint16>, default: empty)

        The list of TCP ports to open as a TCP server.
        NOTE: These ports will enter the "pending" state until a TCP client connects.

* **`~/tcp_client_ports`** (vector<uint16>, default: empty)

        The list of TCP ports to open as a TCP client.
        NOTE: These ports will enter the "pending" state until they are able to connect to a TCP server.

* **`~/udp_ports`** (vector<uint16>, default: empty)

        The list of UDP ports to open for communication.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_modem/issues).


[ROS]: http://www.ros.org
