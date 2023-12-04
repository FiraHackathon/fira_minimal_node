# 1 Overview #

This package provides an example of how to implement a trajectory following algorithm in using Tirrex software developments. It takes data coming from an path matching algorithm and odometry in order to compute a command send to robot controller. 

# 2 Node #

### 2.1 Subscribed Topics ###

- path_matching/info (romea_path_msgs::msg::PathMatchingInfo2D)

    This topic is published by path matching node  and provides a lot informations about trajectory geometry (direction, curvature ...) and vehicle pose in Frenet reference frame (curvilinear abscissa, lateral_deviation, course deviation...)

- odometry (depend of the vehicle type)

    This topic is published by the robot controller. According robot kinematic this message is different, it's a :

    -   romea_mobile_base_msgs::msg::OneAxleSteeringMeasure message when the robot has an one axle steering kinematic
    -   romea_mobile_base_msgs::msg::TwoAxleSteeringMeasure message when the robot has an one two steering kinematic
    -   romea_mobile_base_msgs::msg::SkidSteeringMeasure message when the robot has a skid steering kinematic
    -   romea_mobile_base_msgs::msg::OmniteeringMeasure message when the robot has an omni steering kinematic

- joy (sensors_msgs::msg::Joy) 

    This topic is publish by joystick node. It's used to start and stop the trajectory following algorithm.

### 2.2 Published Topics ###

- cmd_output (depend of the vehicle type)

    This topic contains command sended to robot controller. According robot kinenatic this message is different, it's a :
    -   romea_mobile_base_msgs::msg::OneAxleSteeringCommand message when the robot has an one axle steering kinematic
    -   romea_mobile_base_msgs::msg::TwoAxleSteeringCommand message when the robot has an one two steering kinematic
    -   romea_mobile_base_msgs::msg::SkidSteeringCommand message when the robot has a skid steering kinematic
    -   romea_mobile_base_msgs::msg::OmniteeringCommand message when the robot has an omni steering kinematic

### 2.3 Parameters ###

- base

    Characteristics of the robot

- longitudinal_control.minimal_linear_speed(double, default: 0.3)

    When the vehicle is far from the trajectory, the linear speed is automatically adapted to reach the trajectory while traveling as short distance as possible. This means that the command linear speed is reduced but cannot be lower than this limit so that the vehicle can move forward.

- lateral_controls.selected (string, default classic) 

    Selected command law

- lateral_controls.classic

    Configuration of classic control law

- lateral_controls.predictive:

    Configuration of predicitve control law

- setpoint.desired_linear_speed (double; default 1.0)

    Desired linear speed 

- setpoint.desired_lateral_deviation (double; default 0.0)

    Desired lateral deviation 

- cmd_output.message_type (string)

    Command message type, it depends of vehicle kinematic type (see section 2.1)   

- cmd_output.priority (int, default: 0)

    If priority is not equal to zero, the node send a resgistation request to controller cmd_mux node. The priority of the message must be inside  [0, 255]   

- cmd_output.rate(int, default: 10)

    Rate at which the command is send to the controller

