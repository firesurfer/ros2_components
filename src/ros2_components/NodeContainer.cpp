/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "NodeContainer.h"

namespace ros2_components {


NodeContainer::NodeContainer(rclcpp::Node::SharedPtr _ros_node, int64_t _node_id, std::string _name):
    ros_node(_ros_node), node_id(_node_id), node_name(_name)
{

}

NodeContainer::~NodeContainer()
{
    abort = true;
    rclcpp::shutdown();
    if(spinThread)
        spinThread->join();

}

void NodeContainer::spin(std::chrono::nanoseconds timeout)
{
    if(isSpinning)
        throw AlreadySpinningException();
    isSpinning = true;
    if(timeout == std::chrono::nanoseconds(-1))
        rclcpp::spin_some(ros_node);
    else
    {
        executor.add_node(ros_node);
        executor.spin_once(timeout);
        executor.remove_node(ros_node);

    }
    isSpinning = false;
}

void NodeContainer::spinAsync()
{
    if(isSpinning)
        throw AlreadySpinningException();
    auto async_spin = [&]()
    {
        isSpinning = true;
        rclcpp::WallRate loopRate(this->loop_rate);
        while(!abort && rclcpp::ok())
        {
            rclcpp::spin_some(ros_node);
            loopRate.sleep();
        }
        isSpinning = false;
        isSpinningAsync = false;
    };

    spinThread = std::make_shared<std::thread>(async_spin);
    this->isSpinningAsync = true;
}

rclcpp::Node::SharedPtr NodeContainer::getRosNode()
{
    return ros_node;
}

bool NodeContainer::getIsSpinning() const
{
    return isSpinning;
}

bool NodeContainer::getIsSpinningAsync() const
{
    return isSpinningAsync;
}

int NodeContainer::getLoopRate() const
{
    return loop_rate;
}

void NodeContainer::setLoopRate(int value)
{
    loop_rate = value;
}

std::string NodeContainer::getNodeName() const
{
    //TODO difference between name and real node name!
    return ros_node->get_name();
}

std::string NodeContainer::getNodeNameBase() const
{
    return node_name;
}

int64_t NodeContainer::getNodeId() const
{
    return node_id;
}

bool NodeContainer::ok()
{
    return rclcpp::ok();
}

std::vector<rclcpp::PublisherBase::SharedPtr> NodeContainer::getAllPublishers()
{
    return publishers;
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> NodeContainer::getAllSubscriptions()
{
    return subscriptions;
}
}
