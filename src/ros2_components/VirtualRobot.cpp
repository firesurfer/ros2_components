
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

    if (parameter_list_future.wait_for(5_s) != std::future_status::ready)
    {
        throw std::runtime_error("Could not contact parameterServer");
    }
    auto parameter_list = parameter_list_future.get();
    for (auto & name : parameter_list.names) {

        auto parameters = parameters_client->get_parameters({name});
        if (parameters.wait_for(5_s) != std::future_status::ready)
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


