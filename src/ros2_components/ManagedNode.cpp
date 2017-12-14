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
    CLIArgument logArg = {"logpath","Path to the logfile - also enables the logging to a file", &this->logfilePath};
    CLIArgument configArg = {"configpath","Path to a configfile", &this->configfilePath};
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
    exit();
}

void ManagedNode::spinAsync()
{
    if(!isSetup)
        throw NodeNotInitializedException();
    rosNode->spinAsync();
}

void ManagedNode::spin(std::chrono::nanoseconds timeout)
{
    if(!isSetup)
        throw NodeNotInitializedException();
    rosNode->spin(timeout);
}

bool ManagedNode::ok() const
{
    return rosNode->ok();
}

ComponentManager::SharedPtr ManagedNode::getComponentManager()
{
    return this->CompManager;
}

rclcpp::Node::SharedPtr ManagedNode::getRosNode()
{
    if(!isSetup)
        throw NodeNotInitializedException();
    return this->rosNode->getRosNode();
}

int64_t ManagedNode::getNodeId()
{
    return this->rosNode->getNodeId();
}

int ManagedNode::getLoopRate() const
{
    if(!rosNode)
        throw NodeNotInitializedException();
    return rosNode->getLoopRate();
}

void ManagedNode::setLoopRate(int value)
{
    if(!rosNode)
        throw NodeNotInitializedException();
    rosNode->setLoopRate(value);
}

CLIParser& ManagedNode::getCliParser()
{
    return cliParser;
}

ManagedNodeState ManagedNode::getNodeState() const
{
    if(!isSetup)
        return ManagedNodeState::NotInitialized;
    if(isSetup  && rosNode && !rosNode->getIsSpinning() && !rosNode->getIsSpinningAsync())
        return ManagedNodeState::Initialized;
    if(isSetup  && rosNode && rosNode->getIsSpinningAsync() )
        return ManagedNodeState::SpinningAsync;
    if(isSetup   && rosNode && rosNode->getIsSpinningAsync() )
        return ManagedNodeState::SpinningSync;
    if(! rosNode)
        return ManagedNodeState::ExitCalled;

    return ManagedNodeState::Unknown;

}

NodeContainer::SharedPtr ManagedNode::getRosNodeContainer() const
{
    return rosNode;
}

void ManagedNode::doWork()
{

}

void ManagedNode::exit()
{
    if (rosNode)
    {
        LOG(Info) << "Called exit on: " << rosNode->getNodeName() << std::endl;
    }
}

void ManagedNode::setup()
{
    setup(LogLevel::Info);
}


void ManagedNode::setup(LogLevel logLevel)
{
    //Check if we could parse the id argument. Otherwise the default id is 100
    uint64_t id = 100;
    if(id_str != "")
    {
        id = std::stoi(id_str);
    }

    //Create the ros node base on the given node name and the specified id
    auto internal_node = rclcpp::Node::make_shared(nodeName+ std::to_string(id));
    rosNode = std::make_shared<NodeContainer>(internal_node, id, nodeName);
    //Some info before the logger was started
    std::cout << "Started node: " << rosNode->getRosNode()->get_name() << std::endl;
    //Create the entity that represents the node (This is the real base)
    this->nodeEntity = std::make_shared<NodeEntity>(rosNode->getNodeId(), this->nodeName, rosNode->getRosNode());


    //Init logger
    INIT_LOGGER(rosNode->getRosNode());
    //Set loglevel to given loglevel
    LOGLEVEL(logLevel);
    //If the --logfile argument was successfully parsed, set logfilepath (this will enable logging to a file)
    if(this->logfilePath != "")
        simpleLogger::getInstance()->setLogFilePath(this->logfilePath);
    //Print the build time. This only works if the file was rebuilt at the moment
    LOG(Info) <<"Build at: "<< BuildInfo::get_build_date() << std::endl;
    LOG(Info) << "Version: " << BuildInfo::get_build_version() << std::endl;

    //Create Componentmanager with nodeEntity as base
    this->CompManager = std::make_shared<ComponentManager>(this->rosNode->getRosNode());
    this->isSetup = true; //Set setup to true

    //TEMPORARY WORKAROUND -> https://github.com/ros2/rmw_fastrtps/issues/157
    std::this_thread::sleep_for(std::chrono::milliseconds(1900));
}

}


