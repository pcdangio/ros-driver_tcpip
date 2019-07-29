#include "ros_node.h"

// CONSTRUCTORS
ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_modem");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read standard parameters.
    ros::NodeHandle private_node("~");
    std::string param_local_ip;
    private_node.param<std::string>("local_ip", param_local_ip, "192.168.1.243");
    std::string param_remote_ip;
    private_node.param<std::string>("remote_ip", param_remote_ip, "192.168.1.243");
    double param_spin_rate;
    private_node.param<double>("spin_rate", param_spin_rate, 1000);

    // Read connect port parameters.
    std::vector<int> param_tcp_ports = {4000, 4002, 4004};
    //private_node.getParam("tcp_ports", param_tcp_ports);
    std::vector<int> param_udp_ports = {3000, 3002, 3004};
    //private_node.getParam("udp_ports", param_udp_ports);

    // Initialize driver.
    ros_node::m_driver = new driver(param_local_ip,
                                    param_remote_ip,
                                    std::bind(&ros_node::rx_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
                                    std::bind(&ros_node::disconnect_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Set up publishers, subscribers, and services.
    // Also add connections to driver.
    // TCP:
    for(uint32_t i = 0; i < param_tcp_ports.size(); i++)
    {
        // Get local port from vector.
        uint16_t local_port = static_cast<uint16_t>(param_tcp_ports.at(i));

        // Add connection to driver.
        ros_node::m_driver->add_connection(connection_type::TCP, local_port, local_port + 1);

        // RX Publisher:
        // Generate topic name.
        std::stringstream rx_topic;
        rx_topic << ros::this_node::getName() << "/tcp/" << local_port << "/rx";
        // Add new rx publisher to the map.
        ros_node::m_tcp_rx.insert(std::make_pair(local_port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

        // TX Service:
        // Generate topic name.
        std::stringstream tx_topic;
        tx_topic << ros::this_node::getName() << "/tcp/" << local_port << "/tx";
        // Add new tx service server to the map.
        ros_node::m_tcp_tx.insert(std::make_pair(local_port, ros_node::m_node->advertiseService(tx_topic.str(), &ros_node::tcp_tx, this)));
    }
    // UDP:
    for(uint32_t i = 0; i < param_udp_ports.size(); i++)
    {
        // Get local port from vector.
        uint16_t local_port = static_cast<uint16_t>(param_udp_ports.at(i));

        // Add connection to driver.
        ros_node::m_driver->add_connection(connection_type::UDP, local_port, local_port + 1);

        // RX Publisher:
        // Generate topic name.
        std::stringstream rx_topic;
        rx_topic << ros::this_node::getName() << "/udp/" << local_port << "/rx";
        // Add new rx publisher to the map.
        ros_node::m_udp_rx.insert(std::make_pair(local_port, ros_node::m_node->advertise<driver_modem::DataPacket>(rx_topic.str(), 1)));

        // TX Subscriber:
        // Generate topic name.
        std::stringstream tx_topic;
        tx_topic << ros::this_node::getName() << "/udp/" << local_port << "/tx";
        // Add new tx subscriber to the map.
        ros_node::m_udp_tx.insert(std::make_pair(local_port, ros_node::m_node->subscribe(tx_topic.str(), 1, &ros_node::udp_tx, this)));
    }

    // Initialize ros node members.
    ros_node::m_spin_rate = new ros::Rate(param_spin_rate);


    ROS_INFO_STREAM("Modem initialized.");
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_spin_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// METHODS
void ros_node::spin()
{
    ros_node::m_driver->start();
    while(ros::ok())
    {
        // Spin the driver once.
        //ros_node::m_driver->spin_once();

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_spin_rate->sleep();
    }
    ros_node::m_driver->stop();
}

// MESSAGE CALLBACKS
void ros_node::udp_tx(const driver_modem::DataPacketPtr &message)
{

}
bool ros_node::tcp_tx(driver_modem::TCPtxRequest &request, driver_modem::TCPtxResponse &response)
{

}

// CALLBACKS
void ros_node::rx_callback(connection_type type, uint16_t local_port, uint8_t *data, uint32_t length)
{
    // Deep copy data into new DataPacket message.
    driver_modem::DataPacket message;
    message.header.stamp = ros::Time::now();
    for(uint32_t i = 0; i < length; i++)
    {
        message.data.push_back(data[i]);
    }
    // Clear raw data array.
    delete [] data;

    // Publish received message.
    switch(type)
    {
    case connection_type::TCP:
    {
        ros_node::m_tcp_rx.at(local_port).publish(message);
        break;
    }
    case connection_type::UDP:
    {
        ros_node::m_udp_rx.at(local_port).publish(message);
        break;
    }
    }
}
void ros_node::disconnect_callback(connection_type type, uint16_t local_port)
{

}
