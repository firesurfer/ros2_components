#include "rclcpp/rclcpp.hpp"

#include "ros2_components/Robot.h"
#include "ros2_simple_logger/Logger.h"
#include "ros2_components/ComponentManager.h"
#include "gtest/gtest.h"
using namespace ros2_components;

rclcpp::node::Node::SharedPtr node;
bool abortSpin = false;


TEST (CreateRobotTest, CreateRobot)
{
    LOG(Info) << "Creating test Robot with id 100" << std::endl;
    Robot::SharedPtr testRobot = std::make_shared<Robot>(100,false,node,"Robot");
    ASSERT_EQ(100, testRobot->getId());
    ASSERT_EQ("Robot",testRobot->getClassName());
    ASSERT_EQ("Robot100", testRobot->getName());
}
TEST(CreateRobotTest, AddChild)
{
    LOG(Info) << "Creating test Robot with id 100" << std::endl;
    Robot::SharedPtr testRobot = std::make_shared<Robot>(100,false,node,"Robot");

    Robot::SharedPtr childRobot = std::make_shared<Robot>(101,false,node,"Robot");
    testRobot->addChild(childRobot);
    ASSERT_EQ(101, testRobot->getAllChilds()[0]->getId());
    ASSERT_EQ(101, testRobot->getChildById(101)->getId());
    ASSERT_EQ(1, testRobot->countChilds());
    ASSERT_EQ("Robot101", testRobot->getChildById(101)->getName());
}

TEST(CreateRobotTest, AdvertiseRobot)
{
    ComponentManager manager(node);
    LOG(Info) << "Creating test Robot with id 100" << std::endl;
    Robot::SharedPtr testRobot = std::make_shared<Robot>(100,false,node,"Robot");
    LOG(Info) << "Advertising testRobot to the system" << std::endl;
    testRobot->Advertise(AdvertisementType::Enum::New);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    bool success = false;
    ComponentInfo info = manager.GetInfoToId(100, &success);
    ASSERT_EQ(true, success);
    ASSERT_EQ(100, info.id);

}

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
    node = rclcpp::node::Node::make_shared("create_robot_test");
    std::thread spinner(&spin);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    INIT_LOGGER(node);
    LOGLEVEL(Debug);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();


    abortSpin = true;
    spinner.join();

}
