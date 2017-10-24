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


/* ros2 components */
#include "NodeContainer.h"
#include "BuildInfo.h"
#include "ros2_simple_logger/Logger.h"
#include "ComponentManager.h"
#include "CLIParser.h"
#include "NodeEntity.h"
/**
 * @brief The ManagedNode class
 * This class represents a managed lifecycle node.
 *
 */
namespace ros2_components {


typedef enum
{
    NotInitialized = 0,
    Initialized = 1,
    SpinningAsync = 2,
    SpinningSync = 3,
    ExitCalled = 4,
    Unknown = 5

}ManagedNodeState;

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
     * @brief Spin - Spin this node (manually)
     * @throws NodeNotInitializedException
     */
    virtual void Spin(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief SpinAsync
     * Start the spin function in an extra thread
     * @throws NodeNotInitializedException
     */
    virtual void SpinAsync();
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
     * @throws NodeNotInitializedException
     */
    rclcpp::node::Node::SharedPtr GetRosNode();
    /**
     * @brief GetNodeId
     * @return
     */
    int64_t GetNodeId();
    /**
     * @brief GetLoopRate
     * @return Looprate for async spin
     * @throws NodeNotInitializedException
     */
    int GetLoopRate() const;
    /**
     * @brief SetLoopRate
     * @param value - loop rate for async spin
     */
    void SetLoopRate(int value);
    /**
     * @brief GetCliParser
     * @return
     */
    CLIParser GetCliParser() const;
    /**
     * @brief GetNodeState
     * @return The current state of the node
     */
    ManagedNodeState GetNodeState() const;

    NodeContainer::SharedPtr GetRosNodeContainer() const;

protected:



    /**
     * @brief RosNode
     */
    NodeContainer::SharedPtr RosNode;


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
      * @brief nodeEntity Entity that represents this node.
      */
    NodeEntity::SharedPtr nodeEntity;

    std::string nodeName;


private:


    /**
     * @brief CompManager
     * One component manager per node
     */
    ComponentManager::SharedPtr CompManager;


    std::string id_str= "";



};

}

