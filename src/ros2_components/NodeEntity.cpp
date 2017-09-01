#include "NodeEntity.h"


namespace  ros2_components {


NodeEntity::NodeEntity(uint64_t _id, std::string _name, rclcpp::node::Node::SharedPtr _rosNode):Entity<std_msgs::msg::Empty>(_id,true, _rosNode, "NodeEntity", _name)
{

}

NodeEntity::~NodeEntity()
{

}

}
