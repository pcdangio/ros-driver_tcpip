#include "driver_tcpip.hpp"

int main(int argc, char **argv)
{
    // Initilize ROS.
    ros::init(argc, argv, "driver_tcpip");

    // Create the node.
    driver_tcpip::driver_tcpip_t node;

    // Run the node.
    node.run();
}
