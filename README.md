## ros2_components: An easy to use ros2 component framework

### What is ros2_components

It's an wrapper around the publish and subscribe mechanism of ros2.
The idea is: you don't want to think about topic names and how meta data is exchange, 
but it's nice to represent the hardware of your robot in your software.

The component for ros2 consists at the moment mainly of three files:

Reflect.h
Entity.h
Entity.cpp

Entity is a template base type you can inherit from that generates topic names and uses a parameterclient/server for exchanging meta information. You can now inherit a sensor or an actor that has a certain message type.

For an example how to use it have a look at the examples folder.
