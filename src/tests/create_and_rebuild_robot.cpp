#include "rclcpp/rclcpp.hpp"

rclcpp::node::Node::SharedPtr node;
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::node::Node::make_shared("create_and_rebuild_robot_test");
}
