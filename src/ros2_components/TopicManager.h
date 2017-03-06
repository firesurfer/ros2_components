#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <map>
#include <functional>
namespace  ros2_components {

//See https://discourse.ros.org/t/maximum-amount-of-publisher-subscriptions-per-node/1391
class TopicManager
{
public:

    static TopicManager& getInstance()
    {
        static TopicManager instance;
        return instance;

    }

    template<typename T>
    typename rclcpp::publisher::Publisher<T>::SharedPtr getPublisher(std::string _topicName, rclcpp::node::Node::SharedPtr _rosNode)
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
    typename rclcpp::subscription::Subscription<T>::SharedPtr createSubscription(std::string _topicName, std::function<void(std::shared_ptr<T>)> _callback, rclcpp::node::Node::SharedPtr _rosNode)
    {
        /*for(auto it=subscriptions.begin(); it != subscriptions.end();it++)
        {
            if(it->first == _topicName)
            {
                typename rclcpp::subscription::Subscription<T>::SharedPtr sub = std::dynamic_pointer_cast<rclcpp::subscription::Subscription<T>>(it->second);
                std::vector<function_t> callvec = subscriptionCallbacks[sub];
                auto c_callback = _callback.template target<void (*)(std::shared_ptr<T>)>();
                callvec.push_back(reinterpret_cast<function_t>(c_callback));
                return;
            }
        }
        auto internal_callback = [&](std::shared_ptr<T> msg){
            for(function_t callback: subscriptionCallbacks[subscriptions[_topicName]])
            {

                void (*t_callback)(std::shared_ptr<T>)  = reinterpret_cast<void (*)(std::shared_ptr<T>)>(callback);
                t_callback(msg);
            }
        };*/
        typename rclcpp::subscription::Subscription<T>::SharedPtr sub = _rosNode->create_subscription<T>(_topicName, _callback, rmw_qos_profile_sensor_data);

        subscriptions[_topicName] = sub;
        return sub;
        //subscriptionCallbacks[sub] = std::vector<function_t>();
    }

private:
    TopicManager(){};


    std::map<std::string, rclcpp::publisher::PublisherBase::SharedPtr> publishers;
    std::map<std::string, rclcpp::subscription::SubscriptionBase::SharedPtr> subscriptions;
   // static std::map<rclcpp::subscription::SubscriptionBase::SharedPtr, std::vector<function_t>> subscriptionCallbacks;
};
}


