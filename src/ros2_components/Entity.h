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


#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <functional>
#include <iostream>
#include <mutex>
#include <fstream>
#include <string>



#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "ros2_components_msg/srv/list_childs.hpp"
#include <memory>
#include "Reflect.h"

#include <QObject>
#include <QMetaObject>
#include <QMetaProperty>
using namespace std;
using namespace std::placeholders;
namespace ros2_components
{

class EntityBase : public QObject
{
    Q_OBJECT
public:

    typedef std::shared_ptr<EntityBase> SharedPtr;
    EntityBase(int64_t _id, bool _subscribe, std::shared_ptr< rclcpp::node::Node > _parentNode, std::string _className);



    Q_PROPERTY(bool active READ isActive)
    Q_PROPERTY(int64_t id READ getId)
    Q_PROPERTY(std::string className READ getClassName)
    Q_PROPERTY(bool virtualEntity READ isVirtual)
    /**
     * @brief getId
     * @return The Component ID
     */
    int64_t getId();
    /**
     * @brief getName
     * @return The name of the current component
     * This name is generated from getClassName and getId
     */
    std::string getName();
    /**
     * @brief getClassName
     * @return The class name
     */
    std::string getClassName();
    /**
     * @brief isVirtual
     * @return In case this is a virtual Entity (A Entity without a direct hardware representation) this will be true
     */
    bool isVirtual()
    {
        return virtualEntity;
    }
    /**
     * @brief isSubscriber
     * @return Is this a subscriber or a publisher
     */
    bool isSubscriber()
    {
        return subscriber;
    }
    /**
     * @brief addChild
     * @return adds a child to the childs vector
     */
    virtual void addChild(std::shared_ptr<EntityBase> child);
    /**
     * @brief removeChild Removes the given child
     * @param child child to remote
     */
    virtual void removeChild(std::shared_ptr<EntityBase> child);
    /**
     * @brief getChildById throws exeption if id isnt available
     * @param id
     * @return the child with the given id
     */
    virtual std::shared_ptr<EntityBase> getChildById(int64_t id);
    /**
     * @brief getAllChilds
     * @return all childs of this entity
     */
    virtual std::vector<std::shared_ptr<EntityBase>> getAllChilds(){return childs;}
    /**
     * @brief countChilds
     * @return the number of childs
     */
    virtual uint64_t countChilds();

    /**
     * @brief getParent
     * @return the parent of this entity
     */
    virtual std::shared_ptr<EntityBase> getParent();
    /**
     * @brief publish
     */
    virtual bool publish()=0;
    /**
     * @brief getTopicName
     * @return topic name of the ros2 publisher or subscription
     *
     */
    virtual std::string getTopicName();

    /*
     * Some methods for setting and getting stuff
     */
    virtual bool isActive(){return active;}
    void setActive(bool act){this->active = act;}
    virtual std::string getDescription(){return this->getDescription();}
    virtual void setDescription(std::string des){this->description = des;}

    /**
     * @brief updateParameters
     * update the metainformation (marked with REFLECT) via the parameter Server
     */
    virtual void updateParameters();
    /**
     * @brief publishMetaInformation
     * push the local metainformation (marked with REFLECT) to the parameter Server
     */
    virtual void publishMetaInformation();
    /**
     * @brief IterateThroughAllProperties
     * @param func
     */
    void IterateThroughAllProperties(std::function<void(QMetaProperty)> func);
    bool WasMetaInformationUpdated(){return updated;}
protected:
    /**
      * @brief setParent
      * @param par
      * sets the parent of this entity
      */
     virtual void setParent(std::shared_ptr<EntityBase> par);
    [[deprecated]]
    virtual std::string getAutogeneratedClassName();
    /**
     * @brief parent
     * The node -> we need to pass in order to create the publisher, subscription and parameter client
     */
    std::shared_ptr<rclcpp::node::Node> parentNode;
    /**
     * Helper method so we can use REFLECT on a variable
     */
    template <class T>
    void addElement(string type,T &data)
    {
        internalmap.push_back( (Element*)new SpecificElement<T>(type , data));
    }
    /**
     * Vector all elements that had used REFLECT on them are stored in
     */
    std::vector<Element*> internalmap;

    /**
     * Contains children of this entity -> used for building some kind of tree
     */
    std::vector<std::shared_ptr<EntityBase>> childs;
    /**
     * Parent of this entity
     */
    std::shared_ptr<EntityBase> parent;
    /**
     * @brief pubBase
     */
    std::shared_ptr<rclcpp::publisher::PublisherBase> pubBase;
    /**
     * @brief subBase
     * Baseclass for the subscription -> used for determing topic name
     */
    std::shared_ptr<rclcpp::subscription::SubscriptionBase> subBase;
    /**
     * @brief parameterClient
     * parameterClient used for meta information transport
     */
    std::shared_ptr<rclcpp::parameter_client::AsyncParametersClient> parameterClient;
    /**
     * @brief active
     * is this entity active?
     */
    bool active;
    /**
     * @brief description
     * used defined description
     */
    std::string description;
protected slots:
     virtual void on_child_added(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth);
     virtual void on_child_removed(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth);
signals:
    void childAdded(EntityBase::SharedPtr child,std::shared_ptr<EntityBase> parent, int depth);
    void childRemoved(EntityBase::SharedPtr child,std::shared_ptr<EntityBase> parent, int depth);
    void parametersUpdated();
    void newData();

private:
    /**
     * @brief id
     * Component ID
     */
    int64_t id;
    /**
     * @brief virtualEntity
     * Is it a real component
     */
    bool virtualEntity;
    /**
     * @brief subscriber
     */
    bool subscriber;

    /**
     * @brief Name of the implementing class
     */
    std::string className;
    bool updated = false;

};


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
            pubBase = entityPublisher;
            using namespace std::placeholders;
            parentNode->create_service<ros2_components_msg::srv::ListChilds>(getName()+"_srv", std::bind(&Entity::handleListChildRequest,this,_1,_2,_3));
            //std::cout << "Started service with name:"<< getName()+"_srv"<< std::endl;
        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = parentNode->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
            subBase = entitySubscription;
        }
        std::cout << "Created: " << getName() << " As a subscriber?: " << std::to_string(isSubscriber())<<std::endl;


    }
    /**
     * @brief Copy Constructor
     */
    Entity(const Entity &t):EntityBase(t)
    {

        custom_qos_profile = t.custom_qos_profile;
        this->parameterClient = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(parentNode, "ParameterServer");
        parameterEventSubscription = parameterClient->on_parameter_event(std::bind(&Entity::onParameterEvent, this, _1));
        listeners = t.listeners;


        if(!isSubscriber())
        {
            entityPublisher = parentNode->create_publisher<MessageType>(getName(), custom_qos_profile);
            parentNode->create_service<ros2_components_msg::srv::ListChilds>(getName()+"_srv", std::bind(&Entity::handleListChildRequest,this,_1,_2,_3));
        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = parentNode->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
        }
        //updateParameters();
    }

    virtual ~Entity() {

        this->active = false;
        publishMetaInformation();
    }

    /**
     * @brief publish - Tell class to publish the current data to the world
     * @return
     */
    virtual bool publish()
    {
        std::cout << "Entity:Please override publish function" << std::endl;
        return true;

    }

    /**
     * @brief add a new listener to be called when new data arrives
     */
    void addListener(std::function<void(typename MessageType::SharedPtr)> listener)
    {
        listeners.push_back(listener);
        std::cout << "added listener to: "<< this << std::endl;
    }
    /**
     * @brief tell the word we have new meta information (Like is a lidar mounted upside down)
     */



protected:
    /**
     * @brief This is the method to handle new data inside your entity
     */
    virtual void listenerCallback(const typename MessageType::SharedPtr  msg)
    {
        //To ignore warning

        UNUSED(msg);

    }

    virtual void handleListChildRequest(const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<ros2_components_msg::srv::ListChilds::Request> request,
                                        std::shared_ptr<ros2_components_msg::srv::ListChilds::Response> response)
    {

        UNUSED(request_header);
        UNUSED(request);
        std::vector<std::string> types;
        std::vector<int64_t> ids;

        int i =0;
        for(auto & child: childs)
        {
            response->childtypes.push_back(child->getClassName());
            //std::cout << "Writing child name: "<< child->getName() << " :" <<child->getId()<<std::endl;
            response->childids.push_back(child->getId());
            i++;
        }
        response->listsize = i;

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
            emit newData();
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


