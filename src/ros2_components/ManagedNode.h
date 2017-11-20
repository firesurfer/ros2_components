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
     * @brief doWork
     * Override this function in order to perform tasks that need to be performed more than once.
     */
    virtual void doWork();
    /**
     * @brief exit
     * Call exit in order to stop the spin function and in case Start(true) was called the call of the doWork function.
     * exit gets called in the destructor.
     */
    virtual void exit();
    /**
     * @brief setup
     * Do basic setup tasks after creating a node
     */
    void setup();
    /**
     * @brief setup
     * @param logLevel
     * Initiate the logger with a specific log level instead of LogLevel::Info @see setup
     */
    virtual void setup(LogLevel logLevel);
    /**
     * @brief spin - Spin this node (manually)
     * @throws NodeNotInitializedException
     */
    virtual void spin(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    /**
     * @brief spinAsync
     * Start the spin function in an extra thread
     * @throws NodeNotInitializedException
     */
    virtual void spinAsync();
    /**
     * @brief ok
     * @return If the node is still ok -> Otherwise you should stop spinning!
     */
    virtual bool ok() const;
    /**
     * @brief getComponentManager
     * @return The component manager
     */
    ComponentManager::SharedPtr getComponentManager();
    /**
     * @brief getRosNode
     * @return The encapsulated ros node
     * @throws NodeNotInitializedException
     * @deprecated Use getRosNodeContainer instead!
     */
    [[deprecated("Use getRosNodeContainer instead")]]
    rclcpp::node::Node::SharedPtr getRosNode();
    /**
     * @brief getNodeId
     * @return
     */
    [[deprecated("Use getRosNodeContainer instead")]]
    int64_t getNodeId();
    /**
     * @brief getLoopRate
     * @return Looprate for async spin
     * @throws NodeNotInitializedException
     */
    int getLoopRate() const;
    /**
     * @brief setLoopRate
     * @param value - loop rate for async spin
     */
    void setLoopRate(int value);
    /**
     * @brief getCliParser
     * @return
     */
    CLIParser& getCliParser();
    /**
     * @brief getNodeState
     * @return The current state of the node
     */
    ManagedNodeState getNodeState() const;
    /**
     * @brief getRosNodeContainer
     * @return SharedPtr to the NodeContainer
     */
    NodeContainer::SharedPtr getRosNodeContainer() const;


protected:
    /**
     * @brief rosNode
     */
    NodeContainer::SharedPtr rosNode;
    /**
     * @brief baseEntity
     * The basic entity where the abstraction structure of this node starts from
     */
    EntityBase::SharedPtr baseEntity;
    /**
     * @brief isSetup - was the setup function called
     */
    bool isSetup = false;
    /**
     * @brief logfilePath - Path to the logfile. Parsed from commandline arguments.
     */
    std::string logfilePath;
    /**
     * @brief configfilePath - Path to the configfile. Parsed from commandline arguments.
     */
    std::string configfilePath;
    /**
     * @brief cliParser - Parse for commandline arguments
     */
    CLIParser cliParser;
    /**
      * @brief nodeEntity Entity that represents this node.
      */
    NodeEntity::SharedPtr nodeEntity;

private:

    /**
     * @brief CompManager
     * One component manager per node
     */
    ComponentManager::SharedPtr CompManager;

    /**
     * @brief id_str
     * Helper variable for the parser
     */
    std::string id_str= "";

    /**
     * @brief nodeName
     * used to store the node name until creation of the node in the setup method.
     */
    std::string nodeName;



};

}

