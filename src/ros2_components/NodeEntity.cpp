#include "NodeEntity.h"


namespace  ros2_components {


NodeEntity::NodeEntity(uint64_t _id, std::string _name,  NodeContainer::SharedPtr _nodeContainer):Entity<std_msgs::msg::Empty>(_id,true, _nodeContainer, "NodeEntity", _name)
{
    this->node_name = _nodeContainer->getRosNode()->get_name();
    REFLECT(node_name);
}

NodeEntity::~NodeEntity()
{

}

std::string NodeEntity::getNodeName() const
{
    return node_name;
}

}
