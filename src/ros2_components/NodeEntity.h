#ifndef NODEENTITY_H
#define NODEENTITY_H

#include "Entity.h"
#include "std_msgs/msg/empty.hpp"

namespace ros2_components {

/**
 * @brief The NodeEntity class represents a node in the component system.
 */

class NodeEntity: public Entity<std_msgs::msg::Empty>
{

public:
    typedef std::shared_ptr<NodeEntity> SharedPtr;
    NodeEntity(uint64_t _id, std::string _name, NodeContainer::SharedPtr _nodeContainer);

    virtual ~NodeEntity();

    std::string getNodeName() const;

private:
    std::string node_name;
};
}

#endif // NODEENTITY_H
