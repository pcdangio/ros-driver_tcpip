# driver_tcpip

A ROS package that provides a driver node for TCP/IP communications outside of ROS. It enables sending TCP and/or UDP packets over an existing network interface.

**Author:** Paul D'Angio, pcdangio@gmail.com

**License:** [MIT](LICENSE)

**Contents:**
1. [Installation](#1-installation): Instructions for downloading and building this package.
2. [Usage](#2-usage): How to use the node.

## 1: Installation

### 1.1: Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [driver_tcpip_msgs](https://github.com/pcdangio/ros-driver_tcpip_msgs) (ROS messages for the tcpip driver)

### 1.2: Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```bash
cd catkin_workspace/src
git clone https://github.com/pcdangio/ros-driver_tcpip.git driver_tcpip
cd ../
catkin_make
```

## 2: Usage

### 2.1: Running the Node

Run the driver with the following command:

```bash
rosrun driver_tcpip driver_tcpip
```

### 2.2: Services
|Service|Type|Description|
|---|---|---|
|`~/resolve_ip`|[driver_tcpip_msgs/resolve_ip](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/resolve_ip.srv)|Resolves the IP address of a specified hostname.|
|`~/start_tcp_server`|[driver_tcpip_msgs/start_tcp_server](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/start_tcp_server.srv)|Starts a new TCP server on a specified local endpoint. The server will create new TCP sockets for incoming connections.|
|`~/stop_tcp_server`|[driver_tcpip_msgs/stop_tcp_server](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/stop_tcp_server.srv)|Stops an active TCP server.|
|`~/start_tcp_client`|[driver_tcpip_msgs/start_tcp_client](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/start_tcp_client.srv)|Starts a new TCP client on a specified local endpoint. The client will attempt to connect to the specified remote endpoint.|
|`~/stop_tcp_client`|[driver_tcpip_msgs/stop_tcp_client](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/stop_tcp_client.srv)|Stops an active TCP client.|
|`~/open_udp_socket`|[driver_tcpip_msgs/open_udp_socket](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/open_udp_socket.srv)|Opens a UDP socket at a specified local endpoint.|
|`~/close_socket`|[driver_tcpip_msgs/close_socket](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/close_socket.srv)|Closes the specified TCP or UDP socket.|

### 2.3: Published Topics
|Topic|Type|Description|
|---|---|---|
|`~/status`|[driver_tcpip_msgs/status](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/msg/status.msg)|Indicates the current status of all active TCP servers, TCP clients, and open TCP/UDP sockets.|

### 2.4: Data Transmission
When a TCP or UDP socket is opened, special topics and services are created that allow sending and receiving data through the socket. Each socket is assigned a unique ID, referenced as `{SOCKET_ID}` in the tables below.

**TCP:**

|Topic/Service|Type|Description|
|---|---|---|
|`~/sockets/{SOCKET_ID}/tx`|[driver_tcpip_msgs/send_tcp](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/srv/send_tcp.srv)|A service that sends a TCP packet over the socket specified by `{SOCKET_ID}`. The service succeeds only if the TCP message was sent and recieved.|
|`~/sockets/{SOCKET_ID}/rx`|[driver_tcpip_msgs/tcp_packet](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/msg/tcp_packet.msg)|Published any time a new TCP packet is received from the socket specified by `{SOCKET_ID}`|

**UDP:**

|Topic|Type|Description|
|---|---|---|
|`~/sockets/{SOCKET_ID}/tx`|[driver_tcpip_msgs/udp_packet](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/msg/udp_packet.msg)|A subscriber that sends a packet of data over the UDP socket specified by `{SOCKET_ID}`.|
|`~/sockets/{SOCKET_ID}/rx`|[driver_tcpip_msgs/udp_packet](https://github.com/pcdangio/ros-driver_tcpip_msgs/blob/main/msg/udp_packet.msg)|Published any time a new UDP packet is received from the socket specified by `{SOCKET_ID}`|