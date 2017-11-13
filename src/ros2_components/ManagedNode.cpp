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

    //Save node name for later use
    nodeName = _nodeName;

    //TODO put additional help information into parser

    CLIArgument idArg = {"id","Specify id used for the node", &id_str};
    CLIArgument logArg = {"logpath","Path to the logfile - also enables the logging to a file", &this->LogfilePath};
    CLIArgument configArg = {"configpath","Path to a configfile", &this->ConfigfilePath};
    cliParser.addArgument(idArg);
    cliParser.addArgument(logArg);
    cliParser.addArgument(configArg);

    //This defaults to true. You can pass parseCli = false to the constructor in case you want to add your own arguments in an inherited node.
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
        throw NodeNotInitializedException();
    RosNode->SpinAsync();
}

void ManagedNode::Spin(std::chrono::nanoseconds timeout)
{
    if(!isSetup)
        throw NodeNotInitializedException();
    RosNode->Spin(timeout);
}

bool ManagedNode::Ok() const
{
    return RosNode->Ok();
}

ComponentManager::SharedPtr ManagedNode::GetComponentManager()
{
    return this->CompManager;
}

rclcpp::node::Node::SharedPtr ManagedNode::GetRosNode()
{
    if(!isSetup)
        throw NodeNotInitializedException();
    return this->RosNode->GetRosNode();
}

int64_t ManagedNode::GetNodeId()
{
    return this->RosNode->GetNodeId();
}

int ManagedNode::GetLoopRate() const
{
    if(!RosNode)
        throw NodeNotInitializedException();
    return RosNode->GetLoopRate();
}

void ManagedNode::SetLoopRate(int value)
{
    if(!RosNode)
        throw NodeNotInitializedException();
    RosNode->SetLoopRate(value);
}

CLIParser& ManagedNode::GetCliParser()
{
    return cliParser;
}

ManagedNodeState ManagedNode::GetNodeState() const
{
    if(!isSetup)
        return ManagedNodeState::NotInitialized;
    if(isSetup  && RosNode && !RosNode->GetIsSpinning() && !RosNode->GetIsSpinningAsync())
        return ManagedNodeState::Initialized;
    if(isSetup  && RosNode && RosNode->GetIsSpinningAsync() )
        return ManagedNodeState::SpinningAsync;
    if(isSetup   && RosNode && RosNode->GetIsSpinningAsync() )
        return ManagedNodeState::SpinningSync;
    if(! RosNode)
        return ManagedNodeState::ExitCalled;

    return ManagedNodeState::Unknown;

}

NodeContainer::SharedPtr ManagedNode::GetRosNodeContainer() const
{
    return RosNode;
}

void ManagedNode::DoWork()
{

}

void ManagedNode::Exit()
{
    LOG(Info) << "Called exit on: " << RosNode->GetNodeName() << std::endl;

}

void ManagedNode::Setup()
{
    Setup(LogLevel::Info);
}

void ManagedNode::Setup(LogLevel logLevel)
{
    //Check if we could parse the id argument. Otherwise the default id is 100
    uint64_t id = 100;
    if(id_str != "")
    {
        id = std::stoi(id_str);
    }

    //Create the ros node base on the given node name and the specified id
    auto internal_node = rclcpp::node::Node::make_shared(nodeName+ std::to_string(id));
    RosNode = std::make_shared<NodeContainer>(internal_node, id, nodeName);
    //Some info before the logger was started
    std::cout << "Started node: " << RosNode->GetRosNode()->get_name() << std::endl;
    //Create the entity that represents the node (This is the real base)
    this->nodeEntity = std::make_shared<NodeEntity>(RosNode->GetNodeId(), this->nodeName, RosNode->GetRosNode());


    //Init logger
    INIT_LOGGER(RosNode->GetRosNode());
    //Set loglevel to given loglevel
    LOGLEVEL(logLevel);
    //If the --logfile argument was successfully parsed, set logfilepath (this will enable logging to a file)
    if(this->LogfilePath != "")
        simpleLogger::getInstance()->setLogFilePath(this->LogfilePath);
    //Print the build time. This only works if the file was rebuilt at the moment
    LOG(Info) <<"Build at: "<< BuildInfo::get_build_date() << std::endl;
    LOG(Info) << "Version: " << BuildInfo::get_build_version() << std::endl;

    //Create Componentmanager with nodeEntity as base
    this->CompManager = std::make_shared<ComponentManager>(this->RosNode->GetRosNode());
    this->isSetup = true; //Set setup to true

    //TEMPORARY WORKAROUND -> https://github.com/ros2/rmw_fastrtps/issues/157
    std::this_thread::sleep_for(std::chrono::milliseconds(1900));
}

}


