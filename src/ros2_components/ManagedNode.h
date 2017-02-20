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

/*Qt5*/
#include <QObject>
/* std lib */
#include <iostream>
#include <memory>
#include <thread>
/* ros 2 */
#include "rclcpp/rclcpp.hpp"
/* ros2 components */
#include "ros2_simple_logger/Logger.h"
#include "ComponentManager.h"

/**
 * @brief The ManagedNode class
 * This class represents a managed lifecycle node.
 *
 * Example Usage:
 *
 * Inherit from ManagedNode (MyManagedNode and override DoWork and Setup
 * In the Setup function you should create your the correct BaseEntity (In most cases your robot frame)
 *
 * Afterwards in your main function:
 *
 * std::shared_ptr<MyManagedNode> node = std::make_shared<MyManagedNode>(<nodename>, argc, argv);
 * node->Setup(..);
 *
 * //The following lines are either or
 * //For asynchronous spinning:
 * node->Start();
 * while(rclcpp::ok())
 * {
 *     node->DoWork();
 * }
 *
 * //For asnychronous spinning and working:
 * node->Start(true);
 * while(rclcpp::ok())
 * {
 *     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 * }
 *
 * //For synchronous spinning:
 * while(rclcpp::ok())
 * {
 *      node->Spin();
 *      node->DoWork();
 *      std::this_thread::sleep_for(std::chrono::milliseconds(50));
 *      //Or use rclcpp::loop_rate
 * }
 */
namespace ros2_components {


class ManagedNode: public QObject
{
    Q_OBJECT
public:
    ManagedNode(std::string nodeName,int argc, char* argv[]);
    virtual ~ManagedNode();
    /**
     * @brief DoWork
     * Override this function in order to perform tasks that need to be performed more than once.
     */
    virtual void DoWork();
    /**
     * @brief Exit
     * Call Exit in order to stop the spin function and in case Start(true) was called the call of the DoWork function.
     * Exit gets called in the destructor.
     */
    virtual void Exit();
    /**
     * @brief Start
     * @param multithreaded
     * Start the spin function and in case mutlithreaded==true DoWork will be called in a seperate thread.
     * For synchronous spinning call Spin yourself and dont call start
     */
    virtual void Start(bool multithreaded = true);
    /**
     * @brief Setup
     * Do basic setup tasks after creating a node
     */
    void Setup();
    /**
     * @brief Setup
     * @param logLevel
     * Initiate the logger with a specific log level instead of LogLevel::Info @see Setup
     */
    virtual void Setup(LogLevel logLevel);
    /**
     * @brief Spin
     */
    virtual void Spin();
    /**
     * @brief GetComponentManager
     * @return The component manager
     */
    ComponentManager::SharedPtr GetComponentManager();

protected:
    /**
     * @brief CommandLineArguments
     * Contains the given command line arguments
     */
    std::vector<std::string> CommandLineArguments;

    /**
     * @brief Abort
     * Will be set to true in case Exit was called
     */
    bool Abort = false;
    /**
     * @brief RosNode
     */
    rclcpp::node::Node::SharedPtr RosNode;
    /**
     * @brief NodeId
     */
    int64_t NodeId = 0;
    /**
     * @brief CompManager
     * One component manager per node
     */
    ComponentManager::SharedPtr CompManager;
    /**
     * @brief BaseEntity
     * The basic entity where the abstraction structure of this node starts from
     */
    EntityBase::SharedPtr BaseEntity;
private:
    std::shared_ptr<std::thread> SpinThread;
    std::shared_ptr<std::thread> WorkThread;
    void AsyncWorker();

};

}

