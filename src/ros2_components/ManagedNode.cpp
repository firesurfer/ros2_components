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

#include "ManagedNode.h"
//using namespace std::chrono_literals;
namespace ros2_components {


ManagedNode::ManagedNode(std::string _nodeName, int argc, char *argv[], bool parseCli):cliParser{argv,argc,_nodeName}
{

    //Initialise ros2
    rclcpp::init(argc, argv);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    nodeName = _nodeName;

    //TODO put additional help information into parser

    CLIArgument idArg = {"id","Specify id used for the node", &id_str};
    CLIArgument logArg = {"logpath","Path to the logfile - also enables the logging to a file", &this->LogfilePath};
    CLIArgument configArg = {"configpath","Path to a configfile", &this->ConfigfilePath};
    cliParser.addArgument(idArg);
    cliParser.addArgument(logArg);
    cliParser.addArgument(configArg);

    //This defaults to true. You could pass parseCli = false to the constructor in case you want to add your own arguments in an inherited node.
    if(parseCli)
    {
        cliParser.parse();
    }

}
ManagedNode::~ManagedNode()
{
    Exit();
}

void ManagedNode::SpinAsync()
{
    if(!isSetup)
        throw std::runtime_error("Node wasn't setup yet");
    auto async_spin = [&]()
    {
        isSpinning = true;
        rclcpp::WallRate loopRate(this->loopRate);
        while(!abort && Ok())
        {
            rclcpp::spin_some(RosNode);
            loopRate.sleep();

        }
        isSpinning = false;
    };

    SpinThread = std::make_shared<std::thread>(async_spin);
    this->isSpinningAsync = true;
}
void ManagedNode::Spin(std::chrono::nanoseconds timeout)
{
    if(!isSetup)
        throw std::runtime_error("Node wasn't setup yet");
    isSpinning = true;
    if(timeout == std::chrono::nanoseconds(-1))
        rclcpp::spin_some(RosNode);
    else
    {
        executor->add_node(RosNode);
        executor->spin_once(timeout);
        executor->remove_node(RosNode);

    }
    isSpinning = false;
}


bool ManagedNode::Ok() const
{
    //TODO add &&abort ?
    return rclcpp::ok();
}

ComponentManager::SharedPtr ManagedNode::GetComponentManager()
{
    return this->CompManager;
}

rclcpp::node::Node::SharedPtr ManagedNode::GetRosNode()
{
    return this->RosNode;
}

int64_t ManagedNode::GetNodeId()
{
    return this->NodeId;
}


int ManagedNode::GetLoopRate() const
{
    return loopRate;
}

void ManagedNode::SetLoopRate(int value)
{
    loopRate = value;
}

CLIParser ManagedNode::GetCliParser() const
{
    return cliParser;
}

ManagedNodeState ManagedNode::GetNodeState() const
{
    if(!isSetup)
        return ManagedNodeState::NotInitialized;
    if(isSetup && !isSpinning && !isSpinningAsync && !abort)
        return ManagedNodeState::Initialized;
    if(isSetup && isSpinningAsync && !abort)
        return ManagedNodeState::SpinningAsync;
    if(isSetup && !isSpinningAsync && !abort)
        return ManagedNodeState::SpinningSync;
    if(abort)
        return ManagedNodeState::ExitCalled;

    return ManagedNodeState::Unknown;

}




void ManagedNode::DoWork()
{

}

void ManagedNode::Exit()
{
    LOG(Info) << "Called exit on: " << RosNode->get_name() << std::endl;
    abort = true;
    if(SpinThread)
        SpinThread->join();
    rclcpp::shutdown();
}


void ManagedNode::Setup()
{
    Setup(LogLevel::Info);
}
void ManagedNode::Setup(LogLevel logLevel, bool no_executor)
{
    uint64_t id = 100;
    if(id_str != "")
    {
        std::cout << id_str << std::endl;
        id = std::stoi(id_str);
    }

    //Create the ros node base on the given node name and the specified id
    RosNode = rclcpp::node::Node::make_shared(nodeName+ std::to_string(id));
    NodeId = id;
    std::cout << "Started node: " << RosNode->get_name() << std::endl;

    this->nodeEntity = std::make_shared<NodeEntity>(NodeId, this->nodeName, RosNode);
    //If you want to implement a hardware node create base entity in derived class
    //Check if rosnode is valid -> should be always the case
    if( !this->RosNode)
        throw std::runtime_error("RosNode may not be null - cant proceed with setup");

    //Init logger
    INIT_LOGGER(RosNode);
    //Set loglevel to given loglevel
    LOGLEVEL(logLevel);
    //If the --logfile argument was successfully parsed, set logfilepath (this will enable logging to a file)
    if(this->LogfilePath != "")
        simpleLogger::getInstance()->setLogFilePath(this->LogfilePath);
    LOG(Info) <<"Build at: "<< BuildInfo::get_build_date() << std::endl;


    //Create Componentmanager with nodeEntity as base
    this->CompManager = std::make_shared<ComponentManager>(this->RosNode,this->nodeEntity,true);
    this->isSetup = true; //Set setup to true
}

}


