/*
 * Copyright 2016 <copyright holder> <email>
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

#ifndef ROBOT_H
#define ROBOT_H
#include "ros2_components/Entity.h"
#include "std_msgs/msg/empty.hpp"


#include "ConsoleColor.h"


namespace ros2_components
{

class Robot :  public Entity<std_msgs::msg::Empty>
{
    Q_OBJECT
public:
    typedef  std::shared_ptr<Robot> SharedPtr ;
    Robot( int64_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode, std::string  _name):Entity(_id,_subscribe,parentNode,_name)
    {

    }

    template<typename T>
    std::vector<std::shared_ptr<T>> GetAllChildsOfType()
    {
        std::vector<std::shared_ptr<T>> elements;
        std::function<void(std::shared_ptr<EntityBase>)> GoThrough = [&] (std::shared_ptr<EntityBase> _parent)
        {
            for(auto & child : _parent->getAllChilds())
            {
                std::shared_ptr<T> casted_child = dynamic_pointer_cast<T>(child);
                if(casted_child != NULL)
                {
                    elements.push_back(casted_child);
                }
                GoThrough(child);
            }
        };
        return elements;
    }

    void IterateThroughAllChilds(std::function<void(EntityBase::SharedPtr)> func)
    {
        std::function<void(EntityBase::SharedPtr)> rec_func = [&](EntityBase::SharedPtr base){
            for(auto &  child : base->getAllChilds())
            {
                func(child);
                rec_func(child);

            }

        };
        for(auto & seg: getAllChilds())
        {
            func(seg);
            rec_func(seg);
        }
    }
    template<typename T>
    void IterateThroughAllChildsOfType(std::function<void(std::shared_ptr<T>)> func)
    {
        auto callbackFunc = [&](EntityBase::SharedPtr item)
        {
            std::shared_ptr<T> ch = dynamic_pointer_cast<T>(item);
            if(ch != NULL)
                func(ch);

        };
        IterateThroughAllChilds(callbackFunc);
    }


    void virtual  PrintTree()
    {
        int printDepth= 0;
        std::function<void(EntityBase*)> Print = [&] (EntityBase* ent)
        {
            printDepth++;
            for(auto & child: ent->getAllChilds())
            {
                for(int i = 0; i< printDepth;i++)
                {
                    if(i < printDepth-2)
                        std::cout << " ";
                    else
                        std::cout << "-";
                }
                std::cout  << printMyColor(ConsoleColor::FG_GREEN)<< child->getName()  << printMyColor(ConsoleColor::FG_DEFAULT)<< " Topic: " << child->getTopicName()<< " Previous: " << ent->getName()<< std::endl;

                Print(child.get());
                printDepth--;
            }
        };

        std::cout <<"-"<< printMyColor(ConsoleColor::FG_BLUE) << this->getName() << printMyColor(ConsoleColor::FG_DEFAULT) << std::endl;
        printDepth = 2;

        Print(this);

    }

    static std::vector<int64_t> ListKnownRobots(std::shared_ptr<rclcpp::node::Node> _parentNode,std::string prefix = "")//rclcpp::parameter_client::AsyncParametersClient::SharedPtr parameters_client )
    {
        auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(_parentNode,"ParameterServer");
        std::vector<int64_t> robotIds;
        std::vector<std::string> possiblePrefixes;
        for(int i = 100; i < 20000;i++)
        {
            possiblePrefixes.push_back(prefix+"Robot"+ std::to_string(i));
        }
        auto parameter_list_future = parameters_client->list_parameters(possiblePrefixes, 10);

        if (rclcpp::spin_until_future_complete(_parentNode, parameter_list_future) !=
                rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            throw std::runtime_error("Could not contact parameterServer");
        }
        auto parameter_list = parameter_list_future.get();
        for (auto & name : parameter_list.names) {

            auto parameters = parameters_client->get_parameters({name});
            if (rclcpp::spin_until_future_complete(_parentNode, parameters) !=
                    rclcpp::executor::FutureReturnCode::SUCCESS)
            {

                throw std::runtime_error("Could not contact parameterServer");
            }

            for (auto & parameter : parameters.get()) {
                //std::cout << "Parameter name: " << parameter.get_name() << std::endl;
                //std::cout << "Parameter value (" << parameter.get_type_name() << "): " <<
                //parameter.value_to_string() << std::endl;

                if(parameter.get_name().find(".id") != std::string::npos)
                {

                    int64_t myId = parameter.as_int();
                    auto activePar = parameters_client->get_parameters({prefix+"Robot"+std::to_string(myId)+".active"});
                    if(rclcpp::spin_until_future_complete(_parentNode, activePar) != rclcpp::executor::FutureReturnCode::SUCCESS)
                        throw std::runtime_error("Could not contact parameterServer");
                    if(activePar.get().size() <1)
                        throw std::runtime_error("Could not contact parameterServer");
                    if(activePar.get()[0].as_bool())
                    {
                        robotIds.push_back(myId);
                        std::cout << "Found robot with id: " << myId << std::endl;
                    }


                }
            }
        }

        return robotIds;
    }
 signals:
   void hardwareConnection(EntityBase::SharedPtr seg);
protected:
    std::string printMyColor(ConsoleColor color)
    {
        std::string col = "\033[" + std::to_string(color) + "m";
        return col;
    }
private:



   //std::vector<std::function<void(int)>> onConnectedCallbacks;


};

}
#endif // ROBOT_H
