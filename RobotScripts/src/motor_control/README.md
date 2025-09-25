# Motor Control - ROS 2 Package

## Overview

This package provides functionality to control a bi-directional motor using PWM and the lgpio library.

The node motor\_node subscribes to the '/cmd\_vel' topic for control signals.
Upon recieving such a signal, the appropriate motor control action is applied. 

## Topics

/cmd\_vel | Type: geometry\_msgs/msg/Twist | Description: Provides control signals to effec the velocity of the roboto.
