## ros2_components: An easy to use ros2 component framework

The idea behind ros2_components is to represent the hardware of your robot in a virtual model. Let's 
take a simple robot consisting of a base chasis with two motors, two encoders and a ultrasonic distance sensor.

A model based on it's hardware could be as followed:

```
Robot
└───Motor Unit 1
    └───Motor 1
    └───Encoder 1
└───Motor Unit 2
    └───Motor 2
    └───Encoder 2
└───Ultrasonic 1

```

You could simply use some C++ classes and create for example a base class called component or entity and inherit the robot class and all the other classes from it. Then create a mechanism to push changes, for example the velocity on the motor to the hardware, and the other way round. (For example get updates for the ultrasonic sensor).

On this model it would be easy to run any control algorithms on. But what if you want to have a distributed archicture or even want to control multiple robots (or independend robot parts)? Exactly for this case the ros2_components where created. 

It provides a simple, but powerful mechanism to list all components in the system, rebuild them, and work on the rebuild component like on the original model. The synchronisation in between is done in the ros2_components via ROS2. 



