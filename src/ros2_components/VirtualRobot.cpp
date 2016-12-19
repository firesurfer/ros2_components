/*
 * Copyright 2016 <Lennart Nachtigall> <firesurfer65@yahoo.de>
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
#include "VirtualRobot.h"

std::vector<int64_t> ros2_components::VirtualRobot::ListKnownVirtualRobots(std::shared_ptr<rclcpp::node::Node> _parentNode, string prefix)
{
    auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(_parentNode,"ParameterServer");
    std::vector<int64_t> robotIds;
    std::vector<std::string> possiblePrefixes;
    for(int i = 100; i < 20000;i++)
    {
        possiblePrefixes.push_back(prefix+"VirtualRobot"+ std::to_string(i));
    }
    auto parameter_list_future = parameters_client->list_parameters(possiblePrefixes, 10);

    if (parameter_list_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready)
    {
        throw std::runtime_error("Could not contact parameterServer");
    }
    auto parameter_list = parameter_list_future.get();
    for (auto & name : parameter_list.names) {

        auto parameters = parameters_client->get_parameters({name});
        if (parameters.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        {

            throw std::runtime_error("Could not contact parameterServer");
        }

        for (auto & parameter : parameters.get()) {

            if(parameter.get_name().find(".id") != std::string::npos)
            {

                int64_t myId = parameter.as_int();
                auto activePar = parameters_client->get_parameters({prefix+"VirtualRobot"+std::to_string(myId)+".active"});
                if(rclcpp::spin_until_future_complete(_parentNode, activePar) != rclcpp::executor::FutureReturnCode::SUCCESS)
                    throw std::runtime_error("Could not contact parameterServer");
                if(activePar.get().size() <1)
                    throw std::runtime_error("Could not contact parameterServer");
                if(activePar.get()[0].as_bool())
                {
                    robotIds.push_back(myId);
                    std::cout << "Found virtualRobot with id: " << myId << std::endl;
                }


            }
        }
    }

    return robotIds;
}


