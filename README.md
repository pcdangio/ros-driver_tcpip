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

        Publishes (with latching) a list of all active connections manged by the driver each time a new connection is added or an existing connection is removed.

* **`~/PROTOCOL_TYPE/PORT/rx`** ([driver_modem/DataPacket](https://github.com/pcdangio/ros-driver_modem/blob/master/msg/DataPacket.msg))

        Publishes data that has been received over a particular protocol and port.
        PROTOCOL_TYPE: Either "tcp" or "udp" depending on the connection protocol
        PORT: The port number of the connection.

#### Subscribed Topics
* **`~/udp/PORT/tx`** ([driver_modem/DataPacket](https://github.com/pcdangio/ros-driver_modem/blob/master/msg/DataPacket.msg))

        Accepts data to send via UDP over a particular port.
        PORT: The port number of the connection.

#### Services
* **`~/modify_connection`** ([driver_modem/ModifyConnection](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/ModifyConnection.srv))

        Adds or removes a connection to the driver.

* **`~/tcp/PORT/tx`** ([driver_modem/TCPtx](https://github.com/pcdangio/ros-driver_modem/blob/master/srv/TCPtx.srv))

        Accepts data to send via TCP over a particular port.  This is implemented as a service to indicate success.
        PORT: The port number of the connection.


#### Runtime Parameters

* **`~/local_ip`** (string, default: 192.168.1.2)

        The IP address of the local network interface to use for communication.

* **`~/remote_ip`** (string, default: 192.168.1.3)

        The IP address of the remote device to communicate with.

#### Connection Parameters

These parameters are optional and can be used to create TCP and/or UDP connections on node startup.

* **`~/tcp_local_ports`** (vector<uint16>, default: empty)

        The list of local TCP ports to open for connections.
        NOTE: The modem acts as a TCP client and only sends connection requests.  The connection will fail if a TCP server is not at the remote IP.

* **`~/tcp_remote_ports`** (vector<uint16>, default: empty)

        The corresponding list of remote TCP ports to communicate with.
        NOTE: This list must be the same size as the tcp_local_ports parameter.

* **`~/udp_local_ports`** (vector<uint16>, default: empty)

        The list of local UDP ports to open for connections.

* **`~/udp_remote_ports`** (vector<uint16>, default: empty)

        The corresponding list of remote UDP ports to communicate with.
        NOTE: This list must be the same size as the udp_local_ports parameter.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_modem/issues).


[ROS]: http://www.ros.org
