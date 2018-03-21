#ifndef NODECONTAINER_H
#define NODECONTAINER_H


#include "rclcpp/rclcpp.hpp"
#include "ros2_components_exceptions.h"
namespace  ros2_components {


/**
 * @brief The NodeContainer class encapsules a ros node. It provides status information and sync/async methods for spinning the node.
 */
class NodeContainer
{
public:
    typedef std::shared_ptr<NodeContainer> SharedPtr;
    NodeContainer(rclcpp::Node::SharedPtr _ros_node, int64_t _node_id, std::string _name);

    virtual ~NodeContainer();
    /**
     * @brief spin the node synchronous.
     * @param timeout - in case timeout is != -1 it will add the node to an singlethreaded executor and call spin once on the node with the given timeout. Otherwise spin_some is called.
     */
    void spin(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief spinAsync - Spins the node in a second thread by calling spin_some on the node with the given loop rate
     */
    void spinAsync();
    /**
     * @brief getRosNode
     * @return  The internal rosnode
     */
    rclcpp::Node::SharedPtr getRosNode();
    /**
     * @brief getIsSpinning
     * @return True in case the node is spinning at the moment (Is also set to true in case of async spin)
     */
    bool getIsSpinning() const;
    /**
     * @brief getIsSpinningAsync
     * @return True in case the node is spinning async.
     */
    bool getIsSpinningAsync() const;
    /**
     * @brief getLoopRate
     * @return  The loop rate for async spinning
     */
    int getLoopRate() const;
    /**
     * @brief setLoopRate
     * @param value The loop rate for async spinning
     */
    void setLoopRate(int value);

    /**
     * @brief getNodeName
     * @return - the full name of the node. The full name consists of the base name and the id
     */
    std::string getNodeName() const;
    /**
     * @brief getNodeNameBase
     * @return - only the base name
     * Example: MyNode110 would be the full name. The base name would be MyNode
     */
    std::string getNodeNameBase() const;
    /**
     * @brief getNodeId
     * @return
     */
    int64_t getNodeId() const;

    /**
     * @brief ok
     * @return False in case node was destroyed or rclcpp::ok returns false
     */
    bool ok();

    /**
     * @brief Wraps create_publisher method of the node -> Allows accessing all publishers of a node
     */
    template<typename T>
    typename rclcpp::Publisher< T>::SharedPtr create_publisher(std::string name, rmw_qos_profile_t& qos_profile)
    {
        typename rclcpp::Publisher< T>::SharedPtr pub = ros_node->create_publisher<T>(name, qos_profile);
        publishers.push_back(pub);

        return pub;
    }

    /**
     *@brief Wraps create_subscription of the node -> Allows accessing all subscriptions of a node
     */
    template<typename T>
    typename rclcpp::Subscription<T>::SharedPtr create_subscription(std::string name, std::function<void(T)> callback, rmw_qos_profile_t & qos_profile)
    {
        typename rclcpp::Subscription<T>::SharedPtr sub = ros_node->create_subscription<T>(name, callback, qos_profile);
        subscriptions.push_back(sub);
        return sub;
    }

    std::vector<rclcpp::PublisherBase::SharedPtr> getAllPublishers();
    std::vector<rclcpp::SubscriptionBase::SharedPtr> getAllSubscriptions();
private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::vector<rclcpp::PublisherBase::SharedPtr> publishers;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;

    bool isSpinning = false;
    bool isSpinningAsync = false;
    bool abort = false;
    int loop_rate  = 40;

    int64_t node_id;
    std::string node_name;

    std::shared_ptr<std::thread> spinThread;
};

}
#endif // NODECONTAINER_H
