/*
 * Copyright 2016 <copyright holder> <email>
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

#ifndef VIRTUALROBOT_H
#define VIRTUALROBOT_H

#include "Robot.h"

namespace ros2_components
{
    class VirtualRobot
    {

    public:
	typedef enum {Back = 1, Front = 2}RobotInsertPosition;
	typedef std::shared_ptr<VirtualRobot> SharedPtr;
    VirtualRobot( )
	{
	    
	}

    virtual void AddRobot(std::shared_ptr<Robot> robo, RobotInsertPosition pos = Front) = 0;


    virtual std::shared_ptr<Robot> GetRobotById(int64_t id) = 0;

    std::vector<std::shared_ptr<Robot>> virtual GetAllRobots() = 0;

    int  virtual CountRobots() = 0;

	/*Overwriten methods from robot:*/
	

    virtual void PrintTree() = 0;

    protected:
	std::vector<int64_t> RobotOrder;
	
    };
    
}
#endif // VIRTUALROBOT_H
