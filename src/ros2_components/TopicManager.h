#ifndef TOPICMANAGER_H
#define TOPICMANAGER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <map>
#include <functional>
namespace  ros2_components {

//See https://discourse.ros.org/t/maximum-amount-of-publisher-subscriptions-per-node/1391
class TopicManager
{
public:
    TopicManager();

    template<typename T>
    static typename rclcpp::publisher::Publisher<T>::SharedPtr getPublisher(std::string _topicName, rclcpp::node::Node::SharedPtr _rosNode)
    {

        for(auto it=publishers.begin(); it != publishers.end(); it++)
        {
            if(it->first == _topicName)
            {

                typename rclcpp::publisher::Publisher<T>::SharedPtr pub = std::dynamic_pointer_cast<rclcpp::publisher::Publisher<T>>(it->second);
                return pub;
            }
        }
         typename rclcpp::publisher::Publisher<T>::SharedPtr pub = _rosNode->create_publisher<T>(_topicName, rmw_qos_profile_sensor_data);
         publishers[_topicName] = pub;
         return pub;
    }

    template<typename T>
    static typename rclcpp::subscription::Subscription<T>::SharedPtr getSubscription(std::string _topicName, std::function<void(std::shared_ptr<T>)> _callback, rclcpp::node::Node::SharedPtr _rosNode)
    {
        for(auto it=subscriptions.begin(); it != subscriptions.end();it++)
        {
            if(it->first == _topicName)
            {
                typename rclcpp::subscription::Subscription<T>::SharedPtr sub = std::dynamic_pointer_cast<rclcpp::subscription::Subscription<T>>(it->second);
                return sub;
            }
        }
        auto internal_callback = [&](std::shared_ptr<T> msg){};
        //typename rclcpp::subscription::Subscription<T>::SharedPtr sub
    }

private:
    static std::map<std::string, rclcpp::publisher::PublisherBase::SharedPtr> publishers;
    static std::map<std::string, rclcpp::subscription::SubscriptionBase::SharedPtr> subscriptions;
};
}

#endif // TOPICMANAGER_H
