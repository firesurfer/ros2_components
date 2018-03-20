#ifndef FASTRTPSINFO_H
#define FASTRTPSINFO_H


#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"
#include "rmw_fastrtps_cpp/get_subscriber.hpp"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "NodeContainer.h"

namespace ros2_components {


class FastRtpsInfo
{
public:
    FastRtpsInfo();

    static eprosima::fastrtps::Participant * getFastRtpsParticipant(NodeContainer::SharedPtr  _nodeContainer);
    static std::vector<std::string> listFoundParticipants(NodeContainer::SharedPtr _nodeContainer);
    static eprosima::fastrtps::Publisher * getFastRtpsPublisher(rclcpp::PublisherBase::SharedPtr _publisher);
    static eprosima::fastrtps::Subscriber * getFastRtpsSubscription(rclcpp::SubscriptionBase::SharedPtr _subscription);

};
}
#endif // FASTRTPSINFO_H
