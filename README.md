## ros2_components: An easy to use ros2 component framework

### What is ros2_components

It's an wrapper around the publish and subscribe mechanism of ros2.
The idea is: you don't want to think about topic names and how meta data is exchange, 
but it's nice to represent the hardware of your robot in your software.

The component framework for ros2 consists at the moment mainly of some files:

Reflect.h
Entity.h
Entity.cpp
Robot.h
Robot.cpp
Unit.h
Unit.cpp
VirtualRobot.h
VirtualRobot.cpp

Entity is a template base type you can inherit from that generates topic names and uses a parameterclient/server for exchanging meta information. You can now inherit a sensor or an actor that has a certain message type.
Entity simply encapsulates all communication that is done via ros2. 

Furthermore it its possibly to build a hierachy with diffent entities. 
The basic entity in this case is the Robot wich gives you some methods for iterating through the component tree.
If you want to use more than one robot in the system the VirtualRobot is what you are looking for.

For an example have a look at the https://github.com/firesurfer/ros2_components_demo repository.

