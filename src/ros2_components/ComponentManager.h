#ifndef COMPONENTMANAGER_H
#define COMPONENTMANAGER_H

#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "ros2_components_msg/msg/entity_advertisement.hpp"
#include "ComponentInfo.h"
#include "ros2_components/AdvertisementType.h"
#include "ros2_simple_logger/Logger.h"
#include "ros2_components/EntityFactory.h"


#include <cstdlib>
#include <iostream>
#include <ctime>
namespace ros2_components
{
/**
 * @brief The ComponentManager class - Collects component/entity advertisements in the systems and represents orderered to the user
 */
class ComponentManager:public QObject
{
    Q_OBJECT
public:
    typedef std::shared_ptr<ComponentManager> SharedPtr;
    ComponentManager(rclcpp::node::Node::SharedPtr _localNode);
    bool IDAlreadyInUse(int64_t id)
    {
        for(auto & myInfo: Components)
        {
            if(myInfo.id == id)
                return true;
        }
        return false;
    }
    int64_t GetFreeId()
    {
        int64_t id = 0;

        id = std::rand();
        while(IDAlreadyInUse(id))
        {
            id = std::rand();
        }
    }

    /**
     * Rebuild a component from an id
     */
    template<typename T>
    std::shared_ptr<T> RebuildComponent(int64_t id)
    {
        ComponentInfo relavantInfo;
        bool found = false;
        for(auto & info : Components)
        {
            if(info.id == id)
            {
                relavantInfo = info;
                found = true;
            }
        }
        if(!found)
            throw std::runtime_error("Could not find a component with the given id");
        return RebuildComponent<T>(relavantInfo);
    }
    /**
     * Rebuild a component from a ComponentInfo object using the EntityFactory
     */
    template<typename T>
    std::shared_ptr<T> RebuildComponent(ComponentInfo & info)
    {
        if(!EntityFactory::Contains(info.type))
            throw std::runtime_error("Can't auto-rebuild this component - did register it to the EntityFactory");

        QGenericArgument subscribeArg;
        QGenericArgument idArg = Q_ARG(int64_t, info.id);
        //Determine whether it's a sensor or an actor
        if(info.name.find("Sensor") != std::string::npos)
            subscribeArg = Q_ARG(bool, true);
        else
            subscribeArg = Q_ARG(bool, false);
        QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, localNode);

        QObject * obj = EntityFactory::CreateInstanceFromName(info.type,idArg,subscribeArg,nodeArg);
        T* ent = dynamic_cast<T*>(obj);
        if(ent == NULL)
        {
            //Clean up
            delete obj;
            throw std::runtime_error("Could cast created object to the given type - WTF?!");
        }
        std::shared_ptr<T> secEnt(ent);
        return secEnt;

    }
    std::vector<ComponentInfo>& ListComponents()
    {
        return Components;
    }
    //TODO create function that help sorting the component infos

private:
    rclcpp::node::Node::SharedPtr localNode;
    std::shared_ptr<rclcpp::subscription::Subscription<ros2_components_msg::msg::EntityAdvertisement>> AdvertisementSubscription;

    void AdvertisementCallback(ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg);
    std::vector<ComponentInfo> Components;
signals:
    //Qt signals that can be used in order to stay informed about changes in the system
    void NewComponentAvailable(ComponentInfo &info);
    void ComponentDeleted(ComponentInfo &info);
    void ComponentChanged(ComponentInfo &info);

};
}

#endif // COMPONENTMANAGER_H
