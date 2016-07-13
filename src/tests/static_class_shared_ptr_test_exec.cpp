#include "static_class_shared_ptr_test.h"



rclcpp::node::Node::SharedPtr node;
bool abortSpin = false;

void spin()
{
    rclcpp::executors::SingleThreadedExecutor exec;
    std::cout << "spinning" << std::endl;
    rclcpp::WallRate loop_rate(20);
    while(rclcpp::ok() && !abortSpin)
    {
        exec.spin_node_once(node,std::chrono::milliseconds(10));
        loop_rate.sleep();

    }
    std::cout << "stopped spinning node" << std::endl;

}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::node::Node::make_shared("static_class_shared_ptr_test");
    std::thread * spinner = new std::thread(&spin);


    static_class_shared_ptr_test::getInstance().init(node);

    abortSpin = true;
    spinner->join();
}
