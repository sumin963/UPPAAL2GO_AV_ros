# Example of formal verification of path planning software in a ROS environment 
This repository provides a formal model (Uppaal) and a Go-based execution framework for a ROS-based autonomous driving system running on the Jetson TX2 embedded platform.
The implementation is based on the hardware specifications of the F1TENTH autonomous driving platform, and operates in conjunction with a real ROS system by directly invoking ROS middleware.

# Project Overview
Formal verification and analysis of the timing behavior of a ROS-based path planning system on the Jetson TX2 platform.

# System Modeling Features
The entire data flow—from sensor input to control command generation—was modeled with explicit timing constraints.

ros_algo_TA.xml
![image](https://github.com/user-attachments/assets/2a4b22c8-a8b8-4d5d-bef5-f93cb3cd5867)

In this study, each ROS node is modeled as a Node template within the Uppaal model. The main functional modules—Global Planner, Local Planner, Obstacle Detection, and Controller—are each represented as individual instances.
Callback processing for each node is modeled separately using CallbackQueue and rosSpin, accurately reflecting ROS’s asynchronous event-handling structure.
Execution delays are explicitly represented in Uppaal using delay constructs and clocks. For example, the waiting time between message reception and callback execution is modeled as a bounded delay specified in the CallbackQueue, while the execution itself is expressed in rosSpin using guards and invariants to capture timing constraints.

Within each node, scheduling strictly adheres to ROS’s non-preemptive FIFO policy based on the callback queue mechanism, where messages are processed in the order they arrive.
For inter-node communication, message passing is modeled using subQueues, each of which is centrally managed by a dedicated Policy_FIFO automaton to enforce FIFO delivery across nodes.

ROS1 faces challenges in meeting hard real-time requirements due to its single-threaded ros::spin() execution model and the non-deterministic nature of message queue processing.
In particular, the processing order of the callback queue may not exactly match the message arrival order, and execution times can be difficult to predict.
This study aims to formally analyze these limitations using a formal model, and leverages Uppaal to explicitly define timing constraints and state synchronization, thereby theoretically compensating for the non-deterministic behavior inherent in ROS.
ROS2, with its DDS-based architecture, allows for more flexible Quality of Service (QoS) configurations.
The modeling approach proposed in this study is also extensible to ROS2 environments.

# Related Documents
-*https://github.com/f1tenth
-https://uppaal.org
