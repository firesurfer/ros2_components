#ifndef STATIC_CLASS_SHARED_PTR_TEST_H
#define STATIC_CLASS_SHARED_PTR_TEST_H

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int16.hpp"
#include <mutex>
#include <ostream>
#include <streambuf>
class static_class_shared_ptr_test: public std::ostream, std::streambuf
{
public:

    static static_class_shared_ptr_test& getInstance();
    void init(rclcpp::node::Node::SharedPtr _node);
private:
    static static_class_shared_ptr_test instance;
    static std::mutex test_mutex;
    static_class_shared_ptr_test();
    virtual ~static_class_shared_ptr_test();
    rclcpp::publisher::Publisher<std_msgs::msg::Int16>::SharedPtr publisher;
};

#endif // STATIC_CLASS_SHARED_PTR_TEST_H
