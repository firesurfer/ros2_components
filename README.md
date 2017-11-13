# ros2_components: An object oriented approach to distributed robot systems

In distributed and dynamic robot systems one main task is to track the available hardware and to make sure data from the hardware reaches the controlling parts of the software stack (and vice versa).
Furthermore it is important in case of dynamicly reconfigurable systems to track how the hardware components are in relation to each other. 

The classical approach used in ROS is having a hardware driver, having this driver publishing/listening to some topics and have other nodes that process data coming from the hardware driver (and sending commands to the hardware). ROS1 uses a central hub, the ros core for establishing connections between two nodes. ROS2 took a step forward and uses DDS as a middleware that allows to establishing connections without a central component. It is therefore designated for the use in distributed and dynamic robot systems.

The `ros2_components` were part of my bachelor thesis where I developed a flexible controlstack for the [KAIRO 3](https://www.fzi.de/en/research/projekt-details/kairo/) robot system. (The idea was already developed before for another monolithic robot system) This system consists of several independend segments that can either be controlled all together or in sub parts of two or more segments. The controlstack should allow decoupling KAIRO in 2 or 3 idependent parts, control them, and allow rejoining them. 

In contrast to most current day solutions the ros2_components represent an object oriented approach for hardware abstraction. The idea is that every piece of hardware is modelled in a virtual representation of itself, the so called `Entities`. These virtual representations can be put together to form a virtual copy of the real hardware. 
 
Let's take a simple robot consisting of a base chasis with two motors, two encoders and a ultrasonic distance sensor.

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

But having a model doesn't solve the problem how we can interact from any parts of our system with another part and it doesn't help with the problem of dynamicly changing systems. 
The main feature of the ros2_components is that every child class of the `Entity` class can be rebuild in a distant node using a single command. The clue is that every changes made to the rebuild are transported to the original instance and vice versa without the help of the programmer. 

As an example we use the simple model constructed above:
The real world setup would consist of:

* A node that reads and writes data from/to the hardware and creates a model of the hardware it controls (hardware node)
* A node that implements some algorithms that run on data from the hardware. (dedicated node)

The `hardware node` starts, checks the hardware, creates a model based on the hardware it found. The rest of the time it will read updates from the hardware and write it into the model and takes updates from the model and writes them to the hardware. 

The `dedicated node` on the other hand uses the `ComponentManager` class in order to discover als `Entities` the `hardware node` created before and rebuilds them once all entities has been created. 
It now can work on the rebuilt model like on the real model. 

In case of multiple `hardware nodes` (for controlling multiple hardware parts that are independendly controlled) we can either have:

* Two dedicated nodes that work on the hardware completly independent
* One dedicated node that controls the two hardware parts independendly
* One dedicated node that joins both models into one and controls them like it is one single robot


In case of KAIRO 3 I had two parts that both were controlled by an Intel Nuc (at that time the arm support of ROS2 wasn't that good). The Nucs were connected via wifi. Both ran an hardware node and an dedicated node that allowed them to work as to seperated robots in case parts were disconnected. In case of an physical connection the dedicated nodes decided which node should take the control of both parts. This node then joined both modells into one single model that allowed controlling both parts like one. 

The `ros2_components` have been successfully used with other multi robot systems since then.


