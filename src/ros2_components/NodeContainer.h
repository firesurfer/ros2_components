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
    NodeContainer(rclcpp::node::Node::SharedPtr _ros_node, int64_t _node_id, std::string _name);

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
    rclcpp::node::Node::SharedPtr getRosNode();
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

private:
    rclcpp::node::Node::SharedPtr ros_node;
    rclcpp::executors::SingleThreadedExecutor executor;

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
