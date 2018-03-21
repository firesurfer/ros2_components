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

}

}
