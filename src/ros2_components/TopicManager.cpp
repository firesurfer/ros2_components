#include "TopicManager.h"
namespace ros2_components
{
std::map<std::string, rclcpp::publisher::PublisherBase::SharedPtr> TopicManager::publishers;
std::map<std::string, rclcpp::subscription::SubscriptionBase::SharedPtr> TopicManager::subscriptions;

TopicManager::TopicManager()
{

}
}
