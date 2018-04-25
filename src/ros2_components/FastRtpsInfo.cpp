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

#include "FastRtpsInfo.h"

namespace ros2_components {


FastRtpsInfo::FastRtpsInfo()
{

}

eprosima::fastrtps::Participant *FastRtpsInfo::getFastRtpsParticipant(NodeContainer::SharedPtr _nodeContainer)
{
    rcl_node_t * rcl_node = _nodeContainer->getRosNode()->get_node_base_interface()->get_rcl_node_handle();
    rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
    eprosima::fastrtps::Participant * p = rmw_fastrtps_cpp::get_participant(rmw_node);

    return p;
}

std::vector<std::string> FastRtpsInfo::listFoundParticipants(NodeContainer::SharedPtr _nodeContainer)
{
    auto participant = getFastRtpsParticipant(_nodeContainer);
    return participant->getParticipantNames();
}

eprosima::fastrtps::Publisher *FastRtpsInfo::getFastRtpsPublisher(rclcpp::PublisherBase::SharedPtr _publisher)
{
    rcl_publisher_t * rcl_pub = _publisher->get_publisher_handle();
    rmw_publisher_t * rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
    eprosima::fastrtps::Publisher * p = rmw_fastrtps_cpp::get_publisher(rmw_pub);
    return p;
}

eprosima::fastrtps::Subscriber *FastRtpsInfo::getFastRtpsSubscription(rclcpp::SubscriptionBase::SharedPtr _subscription)
{
    rcl_subscription_t* rcl_sub = _subscription->get_subscription_handle().get();
    rmw_subscription_t* rmw_sub = rcl_subscription_get_rmw_handle(rcl_sub);
    eprosima::fastrtps::Subscriber * s = rmw_fastrtps_cpp::get_subscriber(rmw_sub);
    return s;
}

std::vector<eprosima::fastrtps::Publisher*> FastRtpsInfo::getAllFastRtpsPublishers(NodeContainer::SharedPtr _nodeContainer)
{
   /* auto ros_node =_nodeContainer->getRosNode()->get_node_base_interface()->get_rcl_node_handle();
    auto rmw_node = rcl_node_get_rmw_handle(ros_node);
    auto impl = static_cast<CustomParticipantInfo *>(rmw_node->data);

    WriterInfo * slave_target = impl->secondaryPubListener;
    slave_target->mapmutex.lock();
    for (auto it : slave_target->topicNtypes) {

    }
    slave_target->mapmutex.unlock();*/
    throw std::runtime_error("getAllFastRtpsPublishers not implemented yet");

}

std::string FastRtpsInfo::printPublisherInfo(rclcpp::PublisherBase::SharedPtr _pub)
{

    std::stringstream info_str;
    info_str << "Topic name: " << _pub->get_topic_name() << "\n";
    info_str << "Queue size: " << _pub->get_queue_size() << "\n";

    auto fpub = getFastRtpsPublisher(_pub);
    info_str << "Heartbeat period: " << duration_t_toString(fpub->getAttributes().times.heartbeatPeriod) << "\n";
    info_str << "Initial Hearbeat delay: " << duration_t_toString(fpub->getAttributes().times.initialHeartbeatDelay) << "\n";
    info_str << "Nack Response Delay: s" << duration_t_toString(fpub->getAttributes().times.nackResponseDelay) << "\n";

    return info_str.str();
 }

std::string FastRtpsInfo::printSubscriberInfo(rclcpp::SubscriptionBase::SharedPtr _sub)
{
    std::stringstream info_str;
    info_str << "Topic name: " << _sub->get_topic_name() << "\n";

    auto fsub = getFastRtpsSubscription(_sub);
    info_str << "Unread count: " << fsub->getUnreadCount() << "\n";
    info_str << "Is in clean state: " << fsub->isInCleanState() << "\n";
    info_str << "Hearbeat Response Delay: " << duration_t_toString(fsub->getAttributes().times.heartbeatResponseDelay) << "\n";
    info_str << "Initial Ack Nack Delay: " << duration_t_toString(fsub->getAttributes().times.initialAcknackDelay) << "\n";
    info_str << "User defined ID: " << fsub->getAttributes().getUserDefinedID() << "\n";
    info_str << "Topic data Type: " << fsub->getAttributes().topic.getTopicDataType() << "\n";

    return info_str.str();

 }

std::string FastRtpsInfo::duration_t_toString(eprosima::fastrtps::rtps::Duration_t _dur)
{
    return std::to_string(_dur.seconds) + "." + std::to_string(_dur.fraction);
}

}
