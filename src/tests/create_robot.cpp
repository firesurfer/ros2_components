#include "rclcpp/rclcpp.hpp"

#include "ros2_components/Robot.h"
#include "ros2_simple_logger/Logger.h"
using namespace ros2_components;

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
    //exit(0);
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::node::Node::make_shared("create_robot_test");
    std::thread spinner(&spin);
    INIT_LOGGER(node);
    //LOG(Info) << "Creating test Robot with id 100" << std::endl;
    //Robot::SharedPtr testRobot = std::make_shared<Robot>(100,false,node,"Robot");
   // LOG(Info) << "Advertising testRobot to the system" << std::endl;
   // testRobot->Advertise(AdvertisementType::Enum::New);

    abortSpin = true;
    spinner.join();

}
