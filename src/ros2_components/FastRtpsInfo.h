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

#pragma once


#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include "rclcpp/rclcpp.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"
#include "rmw_fastrtps_cpp/get_subscriber.hpp"
#include "rmw_fastrtps_cpp/custom_participant_info.hpp"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "fastrtps/fastrtps_all.h"
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
    static std::vector<eprosima::fastrtps::Publisher*> getAllFastRtpsPublishers(NodeContainer::SharedPtr _nodeContainer);
    static std::string printPublisherInfo(rclcpp::PublisherBase::SharedPtr _pub);
    static std::string printSubscriberInfo(rclcpp::SubscriptionBase::SharedPtr _sub);
private:
    static std::string duration_t_toString(eprosima::fastrtps::rtps::Duration_t _dur);

};
}

