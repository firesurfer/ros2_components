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


#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <functional>
#include <iostream>
#include <mutex>
#include <fstream>
#include <string>



#include "ros2_simple_logger/Logger.h"
#include "AdvertisementType.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include <memory>
#include "Reflect.h"


#include <QObject>
#include <QMetaObject>
#include <QMetaProperty>


Q_DECLARE_SMART_POINTER_METATYPE(std::shared_ptr)
Q_DECLARE_METATYPE(std::string)

using namespace std;
using namespace std::placeholders;

namespace ros2_components
{

class EntityBase : public QObject, public std::enable_shared_from_this<EntityBase>
{
    Q_OBJECT
public:

    typedef std::shared_ptr<EntityBase> SharedPtr;
    EntityBase(int64_t _id, bool _subscribe, std::shared_ptr< rclcpp::node::Node > _parentNode, std::string _className);
    virtual ~EntityBase();
    //Q_ENUM(AdvertisementType::Enum)
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
    bool isVirtual();
    /**
     * @brief isSubscriber
     * @return Is this a subscriber or a publisher
     */
    bool isSubscriber();
    /**
     * @brief getParentNode
     * @return The rosnode associated with this entity
     */
    rclcpp::node::Node::SharedPtr getParentNode();

    /**
     * @brief addChild
     * @return adds a child to the childs vector
     */
    virtual void addChild(std::shared_ptr<EntityBase> child, bool remote =false);
    /**
     * @brief removeChild Removes the given child
     * @param child child to remote
     */
    virtual void removeChild(std::shared_ptr<EntityBase> child, bool remote = false);
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
    virtual std::string getDescription(){return this->description;}
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
    /**
     * @brief WasMetaInformationUpdated
     * @return
     */
    bool WasMetaInformationUpdated(){return updated;}
    /**
     * @brief IterateThroughAllChilds
     * @param func
     */
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

    /**
     * @brief IterateThroughAllChildsOfType
     */
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
    virtual void on_child_added(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
    virtual void on_child_removed(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
signals:
    void childAdded(EntityBase::SharedPtr child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
    void childRemoved(EntityBase::SharedPtr child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
    void parametersUpdated();
    void newData(EntityBase* ent); //TODO SharedPtr ?

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
    bool advertised = false;


};
}