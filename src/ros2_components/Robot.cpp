
#include "Robot.h"
namespace ros2_components
{
Robot::Robot(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> parentNode, string _name):Entity(_id,_subscribe,parentNode,_name)
{

}

void Robot::RegisterAllChildAddedEvents()
{
    QObject::disconnect(this,&EntityBase::childAdded,this, &Robot::on_child_added);
    QObject::connect(this,&EntityBase::childAdded,this, &Robot::on_child_added);
    auto func = [&](std::shared_ptr<EntityBase> child)
    {
        QObject::disconnect(child.get(), &EntityBase::childAdded,this, &Robot::on_child_added);
        QObject::connect(child.get(), &EntityBase::childAdded,this, &Robot::on_child_added);

    };
    IterateThroughAllChilds(func);

}

void Robot::PrintTree()
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

std::vector<int64_t> Robot::ListKnownRobots(std::shared_ptr<rclcpp::node::Node> _parentNode, string prefix)
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

void Robot::on_child_added(std::shared_ptr<EntityBase> child)
{
    std::cout << "new child was added: " << child->getName() << std::endl;
    RegisterAllChildAddedEvents();
    ros2_components_msg::msg::NewComponentAdded::SharedPtr msg = std::make_shared<ros2_components_msg::msg::NewComponentAdded>();
    msg->componentid =child->getId();
    msg->componenttype= child->getClassName();
    //if(!this->isSubscriber())
       // this->entityPublisher->publish(msg);
}

}
