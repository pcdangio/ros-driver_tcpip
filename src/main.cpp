#include "ros_node.h"

int main(int argc, char **argv)
{
    // Create the node.
    ros_node node(argc, argv);

    // Run the node.
    node.spin();
}
