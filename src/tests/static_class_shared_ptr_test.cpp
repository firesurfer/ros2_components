#include "static_class_shared_ptr_test.h"

static_class_shared_ptr_test static_class_shared_ptr_test::instance;

std::mutex static_class_shared_ptr_test::test_mutex;

static_class_shared_ptr_test::static_class_shared_ptr_test():std::ostream(this)
{

}

static_class_shared_ptr_test::~static_class_shared_ptr_test()
{

}

static_class_shared_ptr_test &static_class_shared_ptr_test::getInstance()
{
    return instance;
}

void static_class_shared_ptr_test::init(rclcpp::node::Node::SharedPtr _node)
{
    std::lock_guard<std::mutex> lock(test_mutex);
    this->publisher = _node->create_publisher<std_msgs::msg::Int16>("static_class_test_pub", rmw_qos_profile_sensor_data);
}
