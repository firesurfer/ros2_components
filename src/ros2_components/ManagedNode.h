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
#include "BuildInfo.h"
#include "ros2_simple_logger/Logger.h"
#include "ComponentManager.h"
#include "CLIParser.h"
#include "NodeEntity.h"
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
 *
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


class ManagedNode
{

public:
    typedef std::shared_ptr<ManagedNode> SharedPtr;
    ManagedNode(std::string nodeName,int argc, char* argv[], bool parseCli = true);
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
     * Start the spin function in an extra thread and in case mutlithreaded==true DoWork will be called in a seperate thread.
     * For synchronous spinning call Spin yourself and dont call start
     */
    virtual void Start(bool multithreaded = false);
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
    virtual void Setup(LogLevel logLevel, bool no_executor= false);
    /**
     * @brief Spin - Spin this node (manually)
     */
    virtual void Spin();
    /**
     * @brief SpinOnce
     */
    virtual void SpinOnce(chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief Ok
     * @return If the node is still ok -> Otherwise you should stop spinning!
     */
    virtual bool Ok() const;
    /**
     * @brief GetComponentManager
     * @return The component manager
     */
    ComponentManager::SharedPtr GetComponentManager();
    /**
     * @brief GetRosNode
     * @return The encapsulated ros node
     */
    rclcpp::node::Node::SharedPtr GetRosNode();
    /**
     * @brief GetNodeId
     * @return
     */
    int64_t GetNodeId();
    /**
     * @brief NodeSetupSuccessfull
     * @return true if setup was called
     */
    bool NodeSetupSuccessfull();

    int GetLoopRate() const;
    void SetLoopRate(int value);

    CLIParser GetCliParser() const;

protected:


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
     * @brief BaseEntity
     * The basic entity where the abstraction structure of this node starts from
     */
    EntityBase::SharedPtr BaseEntity;
    /**
     * @brief isSetup - was the setup function called
     */
    bool isSetup = false;

    /**
     * @brief isSpinningAsync - true in case we called start
     */
    bool isSpinningAsync = false;

    /**
     * @brief LogfilePath - Path to the logfile. Parsed from commandline arguments.
     */
    std::string LogfilePath;
    /**
     * @brief ConfigfilePath - Path to the configfile. Parsed from commandline arguments.
     */
    std::string ConfigfilePath;
    /**
     * @brief cliParser - Parse for commandline arguments
     */
    CLIParser cliParser;

    /**
     * @brief loopRate - contains the rate the spin function of the node is called in case it was started via spin. Can be set during runtime
     */
    int loopRate = 80;
    /**
      * @brief nodeEntity Entity that represents this node.
      */
    NodeEntity::SharedPtr nodeEntity;

    std::string nodeName;
private:
    std::shared_ptr<std::thread> SpinThread;
    std::shared_ptr<std::thread> WorkThread;
    void AsyncWorker();
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

    /**
     * @brief CompManager
     * One component manager per node
     */
    ComponentManager::SharedPtr CompManager;

    std::string id_str= "";



};

}

