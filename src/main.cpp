#include "driver_modem.hpp"

int main(int argc, char **argv)
{
    // Initilize ROS.
    ros::init(argc, argv, "driver_modem");

    // Create the node.
    driver_modem::driver_modem_t node;

    // Run the node.
    node.run();
}
