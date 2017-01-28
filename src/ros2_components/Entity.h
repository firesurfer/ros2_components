#include "EntityBase.h"

namespace ros2_components {

template <typename MessageType>
class Entity : public EntityBase
{

public:

    /**
     * @brief Constructor of Entity
     * @param className is used together with the id to itentify topics, etc. of this entity
     */
    Entity(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> parentNode, std::string className) : EntityBase(_id, _subscribe, parentNode, className)
    {


        //Some ROS2 QOS Configuration -> Taken from an example
        custom_qos_profile = rmw_qos_profile_sensor_data;
        //custom_qos_profile.depth = 2;

        //custom_qos_profile.history = hist_pol;
        //Create a new parameterClient
        //The client is used for storing meta information about a component
        this->parameterClient = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(parentNode, "ParameterServer");

        //Register the client on an event that is thrown in case a parameter has changed
        parameterEventSubscription = parameterClient->on_parameter_event(std::bind(&Entity::onParameterEvent, this, _1));



        if(!isSubscriber())
        {
            entityPublisher = parentNode->create_publisher<MessageType>(getName(), custom_qos_profile);

            //rmw_qos_profile_services_default
            rmw_qos_profile_t component_manager_profile = rmw_qos_profile_parameters;
            component_manager_profile.depth = 1000;
            //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;
            pubBase = entityPublisher;

        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = parentNode->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
            subBase = entitySubscription;
        }

        LOG(LogLevel::Info) << "Created: " << getName() << " As a subscriber?: " << std::to_string(isSubscriber())<<std::endl;


    }


    virtual ~Entity() {

        // LOG(LogLevel::Info) << "Destroying: " << this->getName() << std::endl;
        //this->active = false;
        //publishMetaInformation();
    }

    /**
     * @brief publish - Tell class to publish the current data to the world
     * @return
     */
    virtual bool publish()
    {
        LOG(Error) << "Entity:Please override publish function" << std::endl;
        return true;

    }

    /**
     * @brief add a new listener to be called when new data arrives
     */
    void addListener(std::function<void(typename MessageType::SharedPtr)> listener)
    {
        listeners.push_back(listener);
    }



protected:
    /**
     * @brief This is the method to handle new data inside your entity
     */
    virtual void listenerCallback(const typename MessageType::SharedPtr  msg)
    {
        //To ignore warning

        UNUSED(msg);

    }


    //ROS 2 Stuff
    rmw_qos_profile_t custom_qos_profile;

    std::shared_ptr<rclcpp::publisher::Publisher<MessageType>> entityPublisher;
    std::shared_ptr<rclcpp::subscription::Subscription<MessageType>> entitySubscription;
    rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSubscription;




private:

    std::vector<std::function<void(typename MessageType::SharedPtr)>> listeners;
    /**
     * @brief calls the rest of the registerd listeners
     */
    void internalListenerCallback(const typename MessageType::SharedPtr msg)
    {
        //std::cout << "New message in: "<< getName()<< " ptr: " <<this<< " Listeners: " << listeners.size()<<std::endl;
        if(msg)
        {
            listenerCallback(msg);
            emit newData(this);
            for(auto listener : listeners) {
                listener(msg);
            }
        }

    }

    /**
     * @brief onParameterEvent
     * @param event
     * Gets called in case the meta information of the component has changed
     */
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        //std::cout << "New parameter event in: "<<parentNode->get_name()  << ":" <<getName() << std::endl;
        std::vector< rclcpp::parameter::ParameterVariant> params;
        for (auto & new_parameter : event->new_parameters)
        {
            if(new_parameter.name.find(getName()) != std::string::npos)
                params.push_back(rclcpp::parameter::ParameterVariant::from_parameter(new_parameter));
        }
        for (auto & changed_parameter : event->changed_parameters)
        {
            if(changed_parameter.name.find(getName()) != std::string::npos)
                params.push_back(rclcpp::parameter::ParameterVariant::from_parameter(changed_parameter));
        }
        for (auto & parameter : params)
        {
            //std::cout << parameter.get_name() << std::endl;
            std::string reducedParameter = parameter.get_name();
            reducedParameter.erase(0,reducedParameter.find_last_of(".")+1);
            //std::cout << "Reduced parameter: " << reducedParameter << std::endl;
            for (auto & internal_val : internalmap)
            {
                if(internal_val->key == reducedParameter)
                {


                    /*
                     * uint8 PARAMETER_NOT_SET=0
                     * uint8 PARAMETER_BOOL=1
                     * uint8 PARAMETER_INTEGER=2
                     * uint8 PARAMETER_DOUBLE=3
                     * uint8 PARAMETER_STRING=4
                     * uint8 PARAMETER_BYTES=5
                     */

                    switch(parameter.get_type())
                    {
                    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                    {
                        SpecificElement<bool>* elem = static_cast<SpecificElement<bool>*>(internal_val);
                        elem->setValue(parameter.as_bool());
                        break;
                    }
                    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                    {
                        SpecificElement<int64_t>* elem = static_cast<SpecificElement<int64_t>*>(internal_val);
                        elem->setValue(parameter.as_int());
                        break;
                    }
                    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                    {
                        SpecificElement<double>* elem = static_cast<SpecificElement<double>*>(internal_val);
                        elem->setValue(parameter.as_double());
                        break;
                    }
                    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                    {
                        SpecificElement<std::string>* elem = static_cast<SpecificElement<std::string>*>(internal_val);
                        elem->setValue(parameter.as_string());
                        break;
                    }
                    case rcl_interfaces::msg::ParameterType::PARAMETER_BYTES:
                    {
                        SpecificElement<std::vector<uint8_t>>* elem = static_cast<SpecificElement<std::vector<uint8_t>>*>(internal_val);
                        elem->setValue(parameter.as_bytes());
                        break;
                    }
                    case rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET:
                    {
                        break;
                    }

                    }
                    /*for(auto it = internalmap.begin(); it != internalmap.end(); it++)
                    {
                    (*it)->print();
                    }*/
                }
            }
        }
    }

};

}


