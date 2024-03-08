---
title: "ROS2 Control with the JetBot Part 2: Building a ROS2 Control System"
slug: jetbot-motors-pt2
authors: mike
tags: [aws, robotics, communication, embedded, jetbot, ros2, control]
---

This is the second part of the "ROS2 Control with the JetBot" series, where I show you how to get a JetBot working with ROS2 Control! This is a sequel to the [part 1 blog post](/blog/jetbot-motors-pt2), where I showed how to drive the JetBot's motors using I<sup>2</sup>C and PWM, written in C++.

In this post, I show the next step in making ROS2 Control work with the WaveShare JetBot - wrapping the motor control code in a ROS2 Control System. I'll walk through some ROS2 Control concepts, show the example repository for ROS2 Control implementations, and then show how to implement the System for JetBot and see it running.

This post is also available in video form - check the video link below if you want to follow along!

<!-- TODO: insert video link -->

## ROS2 Control Concepts

First, before talking about any of these concepts, there's an important distinction to make: [ROS Control](http://wiki.ros.org/ros_control) and [ROS2 Control](https://control.ros.org/master/index.html) are *different systems*, and are not compatible with one another. This post is focused on ROS2 Control.



<!-- PLAN
Summary: build library, works with ROS2 control, moves JetBot.
Describe ROS2 Control. Mention ROS1. How it's divided up. Sensor/System/Actuator.
Show ROS2 examples - the starting point for any custom implementation.
Show our system. Show all the parts of the code base and what they do.
Build and run the bastard on the JetBot.
Talk next steps - probably running this guy in simulation. A transform for the state publisher, a Gazebo version of the robot.
-->
