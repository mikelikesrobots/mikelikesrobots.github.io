---
title: "FOSDEM Robotics Dev Room Summary"
slug: fosdem-robotics-2026
authors: mike
tags: [robotics, conference, fosdem]
---

FOSDEM is a conference which is free to attend. This year's conference (2026) was held in Brussels, Belgium over two days. The talks were separated into tracks based on subject, and one of the tracks was Robotics & Simulation. I managed to see almost all of the talks, but the room was constantly full with a long queue - clearly, robotics is popular!

This post is a summary of the talks with links and my own thoughts. If you'd rather go direct to the source, go to the [FOSDEM 2026 website](https://fosdem.org/2026/) (specifically, the [Robotics & Simulation track page](https://fosdem.org/2026/schedule/track/robotics-and-simulation/)).

<!-- truncate -->

## Summary

These are the main trends that I noticed over the whole day of talks:

- ROS 2 remains the backbone of the robotics open source community. It was mentioned in virtually every talk, which is unsurprising given its popularity in the robotics space. Even the talks that presented alternatives to ROS, like [Copper-rs](#copper-rs), compared those alternatives to ROS.

- A surprising theme of the dev room was the use of Rust. It came up in a surprising number of talks, including those that weren't explicitly about Rust libraries like [rclrs](#rclrs) and [ROS-Z](#ros-z). While Rust tools are still new to the scene, they are no longer untested, and are being integrated in many places in open source software. Examples include [use in Gazebo](#a-core-developers-insights-on-gazebos-future) and using pixi as a build tool.

- Speaking of [pixi](https://prefix.dev/), this was another theme, although explicitly mentioned in fewer talks than Rust. Previously, I only knew of it for dependency management, but the speakers demonstrated it being used for packaging and distributing robotics software as well. It seems like a tool that solves many pain points of building for robots.

- Finally, containerisation seems to be the de facto standard for distributing robotics software. [Apptainer](#apptainer) offers an interesting way to run containers and is compatible with Docker, while talks like [Productionising ROS with Bazel](#productionising-ros) showed mature pipelines using Docker alongside hermetic builds.

Conferences like FOSDEM are excellent for getting a view of what the robotics community is excited about. It seems like ROS 2, Rust, pixi, and distributing software using containers are the current trends. I'll be sure to attend FOSDEM 2027 to see if those trends are continuing - hopefully there will be a larger room available to accommodate all the roboticists!

## Special Mentions

The full talk list is below, but I wanted to highlight a few of the talks from the list.

First, [Esteve's talk on `rclrs`](#rclrs) was great to see. I was familiar with a lot of the concepts and arguments, as I've covered `rclrs` in a couple of posts on this blog already (see [Comparing Rust Libraries for ROS Nodes](/blog/ros2-rust-comparison) and [Building a ROS2 node in Rust!](http://localhost:3000/blog/ros2-rust-node)). Still, it was great to hear from its creator about what went into it and how it will be officially supported in the ROS build farm soon. Thank you, Esteve!

Next, the [longer talk from Botronics](#botronics-ixi) was excellent. Seeing the decisions and mistakes that went into a full robot system was fascinating and eye-opening. Not only that, but Botronics will be hosting ROSCon Belgium this year, so make sure to submit papers and buy tickets if you're interested! I'll update this post with links as soon as they're available.

Another talk with some excellent advice was the ["Calibrate good times" talk](#calibrate-good-times) from Roland and Sam. While I haven't developed data collection systems for robots other than for debugging, I still found the advice here excellent, and anyone looking to collect robotics data should give it a watch.

Finally, a couple of talks inspired me to look deeper into the presented material. In the future, I hope to build [the Upkie robot](#upkie) for myself, and I also hope to try [copper-rs](#copper-rs) out on a real robot. In both cases, I'll write up and record my experience to share here. Watch this space or [my YouTube channel](https://youtube.com/@mikelikesrobots) if you're interested to see that.

## Full Talk List

Each talk is listed below with its duration, a reason you might be interested to watch, and information I could get from the speaker's page about their background.

If any speakers would like their information updated, please contact me and let me know!

---

- [Turning a cheap commercial vacuum cleaner into a useful Open Source mapping tool](https://fosdem.org/2026/schedule/event/PDWNCJ-map-hacking-a_cheap-robot-vac-with-open-source-sw/) (25m) - [Stef Dillo](https://fosdem.org/2026/schedule/speaker/stef_dillo/) of [Familiar Robotics](https://familiarrobotics.com/)
    
    **You should watch if:**
    - You work with making maps from *real* sensor data
    - You're interested in open source mapping robots
    - You enjoy reverse engineering products from others!

---

<a id="calibrate-good-times"></a>
- [Calibrate good times! The tools and methods to get top-quality robot data](https://fosdem.org/2026/schedule/event/DJK8WL-calibrate-good-times/) (25m) - [Roland Meertens](https://fosdem.org/2026/schedule/speaker/roland_meertens/) of [Wayve](https://wayve.ai/), [Sam Pfeiffer](https://fosdem.org/2026/schedule/speaker/sam_pfeiffer/) of [Humanoid](https://thehumanoid.ai/)
    
    **You should watch if:**
    - You want excellent advice, built from experience, about gathering and working with clean data from as early as possible
    - You're gathering data, but not making sure it's clean!

---

<a id="copper-rs"></a>
- [Bridging robotics and systems programming: Why Copper-rs is a game changer](https://fosdem.org/2026/schedule/event/SK8EGJ-copper-rust-robotics-runtime/) (25m) - [Guillaume Binet](https://fosdem.org/2026/schedule/speaker/guillaume_binet/) of [Copper Robotics](https://www.copper-robotics.com/)
    
    **You should watch if:**
    - You're interested in alternatives to ROS for robotics frameworks
    - You want to see determinism in your robots
    - You want higher performance than ROS can offer from your robots
    - You like Rust!

---

- [Apptainer: Easy Containerization for Robotics](https://fosdem.org/2026/schedule/event/PHRS33-apptainer-easy-containerization-for-robotics/) (5m) - [Malte Schrader](https://fosdem.org/2026/schedule/speaker/malte_schrader/)
    
    **You should watch if:**
    - You like Docker for packaging software, but want a better way to provide system access to containers

---

- [Just1 - An Open-Source Autonomous Mecanum Wheel Robot](https://fosdem.org/2026/schedule/event/KCPTX7-just1/) (5m) - [Nicolas Rodriguez](https://fosdem.org/2026/schedule/speaker/nicolas_rodriguez/)
    
    **You should watch if:**
    - You want to build a very cheap robot that still has lidar and camera
    - You're interested in mecanum wheels, which allow a 4-wheeled robot to strafe left and right
    - You want a robot with manual and autonomous control out of the box

---

- [Modernizing ROS 2 Skills: Hacking and Orchestrating Cloud Brains, Physical Sensors, and the Network](https://fosdem.org/2026/schedule/event/3SFYWM-hacking-cyber-physical_systems-with-ros2/) (5m) - [Miguel Xochicale](https://fosdem.org/2026/schedule/speaker/miguel_xochicale/) of [University College London](https://www.ucl.ac.uk/)
    
    **You should watch if:**
    - You want to provide access to ROS for multiple users (in this case, students)
    - You want to use a mainframe for ROS and have clients connect into it
    - You want to effectively network between multiple ROS containers

---

- [Benchmarking platform for robot localization systems](https://fosdem.org/2026/schedule/event/HCE8C9-lambkin_benchmarking_for_localization_ekumen/) (5m) - [Júlia Marsal](https://fosdem.org/2026/schedule/speaker/julia_marsal/) of [Ekumen](https://ekumenlabs.com/)
    
    **You should watch if:**
    - You're doing navigation or localisation with your robot and want to explore the best option
    - You want to autonomously test localisation conditions, including in Continuous Integration
    - You want to view detailed metrics from benchmarks generated with "lambkin", the presented software

---

- [rosidlcpp: A Journey Through ROS2 Build Time Optimization](https://fosdem.org/2026/schedule/event/WVUZ3C-rosidlcpp_a_journey_through_ros2_build_time_optimization/) (5m) - [Anthony Welte](https://fosdem.org/2026/schedule/speaker/anthony_welte/)
    
    **You should watch if:**
    - You're interested in the core parts of ROS compilation
    - You're interested in optimisation techniques
    - You want to see where improvements to ROS come from!

---

- [A Core Developer's insights on Gazebo's Future](https://fosdem.org/2026/schedule/event/8HTRVV-a_core_developers_insights_on_gazebos_future/) (45m) - [Jose Luis Rivero](https://fosdem.org/2026/schedule/speaker/jose_luis_rivero/), co-founder of [Honu Robotics](https://www.honurobotics.com) and Project Management Committee member of the Infrastructure and Gazebo project
    
    **You should watch if:**
    - You're interested in the history of Gazebo
    - You want to hear what influences decisions about Gazebo's future, particularly with regards to available technologies
    - You want to know what's in store for Gazebo
    - You want to see Rust being used in more places!

---

<a id="rclrs"></a>
- [Introducing rclrs: the official ROS 2 client library for Rust](https://fosdem.org/2026/schedule/event/J8ZLKG-introducing_rclrs_the_official_ros_2_client_library_for_rust/) (25m) - [Esteve Fernández](https://fosdem.org/2026/schedule/speaker/esteve_fernandez/), an original author of ROS 2
    
    **You should watch if:**
    - You're interested in using Rust for robotics (ROS in particular!)
    - You want to hear from a core ROS 2 developer
    - You want to hear news of Rust being better supported in ROS

---

<a id="upkie"></a>
- [Open-Source Robotics in Practice: Lessons from Upkie Wheeled Bipeds](https://fosdem.org/2026/schedule/event/8PUMMD-open-source-robotics-practice-upkie-wheeled-bipeds/) (25m) - [Stéphane Caron](https://fosdem.org/2026/schedule/speaker/stephane_caron/)
    
    **You should watch if:**
    - You're interested in open source robots in general
        - The speaker maintains [awesome-open-source-robots](https://github.com/stephane-caron/awesome-open-source-robots), a list of open source robots
    - You want to build an open source robot of your own
    - You want to learn practical Reinforcement Learning

---

- [Middleware Pain? Meet iceoryx2](https://fosdem.org/2026/schedule/event/M7TKVG-meet-iceoryx2/) (25m) - [Michael Poehnl](https://fosdem.org/2026/schedule/speaker/michael_poehnl/) of [Ekxide](https://ekxide.io/)
    
    **You should watch if:**
    - You're interested in more ways to move messages around than ROS supports
    - You want to use more advanced events for acting on data
    - You're interested in very high performance robotics, including shared memory transport

---

- [Precision Landing with PX4 and ROS 2 using Aruco Markers](https://fosdem.org/2026/schedule/event/XRE97C-precision_landing_with_px4_and_ros_2_using_aruco_markers/) (25m) - [Ramon Roche](https://fosdem.org/2026/schedule/speaker/ramon_roche/), [Beniamino Pozzan](https://fosdem.org/2026/schedule/speaker/beniamino_pozzan/)
    
    **You should watch if:**
    - You want to see how to integrate PX4 with ROS 2
    - You want to hear about custom controllers in PX4
    - You want to see a demo run successfully right at the last moment!

---

<a id="botronics-ixi"></a>
- [Simple, Safe, Open: Building Your First ROS 2 Rover with Rust and Pixi](https://fosdem.org/2026/schedule/event/3PBHXY-simple_safe_open_building_your_first_ros_2_rover_with_rust_and_pixi/) (10m) - [Christophe Simon](https://fosdem.org/2026/schedule/speaker/christophe_simon/), [Nicolas Daube](https://fosdem.org/2026/schedule/speaker/nicolas_daube/) of [Botronics](https://www.botronics.be/)
    
    **You should watch if:**
    - You're interested in the cheapest possible rover build
    - You want better ways to manage dependencies and export builds
    - You're interested in Rust

---

- [Vehicle Dynamics Sim: accurately and easily simulate actuation limits](https://fosdem.org/2026/schedule/event/QXGHRL-vehicle-dynamics-sim/) (10m) - [Arne Baeyens](https://fosdem.org/2026/schedule/speaker/arne_baeyens/) of [Intermodalics](https://www.intermodalics.ai/)
    
    **You should watch if:**
    - You have issues with vehicle dynamics in commonly-used simulations
    - You want your real robot to behave more closely compared to simulation
    - You want to simulate by only tweaking a few parameters

---

- [Productionising ROS when you have no choice (with Bazel)](https://fosdem.org/2026/schedule/event/MUXVUK-prod-ros-bazel/) (10m) - [Ricardo Delfin](https://fosdem.org/2026/schedule/speaker/ricardo_delfin/) of [Humanoid](https://thehumanoid.ai/)
    
    **You should watch if:**
    - You want hermetic builds for your robots
    - You want to see new, reliable ways of building Docker images for your robots
    - You want to use a production stack for building and deploying robot software reliably

---

- [ArduPilot Advanced Integration](https://fosdem.org/2026/schedule/event/98EYLV-ardupilot_advanced_integration/) (10m) - [Pierre Kancir](https://fosdem.org/2026/schedule/speaker/pierre_kancir/)
    
    **You should watch if:**
    - You're interested in different drone builds
    - You want to see the internals and parts that make up a drone
    - You want to build drones without using ROS

---

- [The Technical Stacks Behind Botronics' iXi Autonomous Golf Trolley](https://fosdem.org/2026/schedule/event/M38A3V-botronics-robotics-tech-stack/) (45m) - [Antoine Van Malleghem](https://fosdem.org/2026/schedule/speaker/antoine_van_malleghem/), [Enzo Ghisoni](https://fosdem.org/2026/schedule/speaker/enzo_ghisoni/), [David Moli](https://fosdem.org/2026/schedule/speaker/david_moli/) of [Botronics](https://www.botronics.be/)
    
    **You should watch if:**
    - You want advice from a company making and selling real robots
    - You want to see the full tech stack of a working robot
    - You want to hear about what went into the tech stack that the company settled on, including the mistakes they made

---

- [ROS-Z: A Rust/Zenoh-native stack, fully ROS 2-compliant](https://fosdem.org/2026/schedule/event/BQ8DVM-ros-z/) (25m) - [Julien Enoch](https://fosdem.org/2026/schedule/speaker/julien_enoch/), [Yuyuan Yuan](https://fosdem.org/2026/schedule/speaker/yuyuan_yuan/) of [ZettaScale](https://www.zettascale.tech/)
    
    **You should watch if:**
    - You're interested in Rust in robotics, particularly ROS
    - You want to use a higher performance version of ROS, as ROS-Z replaces many layers of ROS while remaining compatible with ROS
    - You want to see more from the makers of Zenoh!

---

- [EasyNav: An open-source framework for navigating everywhere](https://fosdem.org/2026/schedule/event/VD7GN8-easynav/) (25m) - [Francisco Martín Rico](https://fosdem.org/2026/schedule/speaker/francisco_martin_rico/), [Francisco Miguel Moreno](https://fosdem.org/2026/schedule/speaker/francisco_miguel_moreno/) of [Universidad Rey Juan Carlos](https://en.urjc.es/)
    
    **You should watch if:**
    - You have navigation cases that are difficult to solve with Nav2
    - You want to use multiple types of maps to navigate with
    - You need to work with multiple elevations in the same map

---

- [AutoAPMS: Lightweight and versatile integration of behavior trees into the ROS 2 ecosystem](https://fosdem.org/2026/schedule/event/RUE39L-auto-apms/) (25m) - [Robin Müller](https://fosdem.org/2026/schedule/speaker/robin_muller/)
    
    **You should watch if:**
    - You want to define behaviour trees in a very flexible way (using CMake)
    - You want to see the power of behaviour trees
    - You enjoy seeing how a simple example can explain complex behaviours

---

- [PlotJuggler: the log visualization tool loved by roboticists](https://fosdem.org/2026/schedule/event/MNT7XJ-plotjuggler_the_log_visualization_tool_loved_by_roboticists/) (25m) - [Davide Faconti](https://fosdem.org/2026/schedule/speaker/davide_faconti/)
    
    **You should watch if:**
    - You want to see a more entertaining talk (I enjoyed the James Bond theme!)
    - You're interested in the future of a very widely-used data visualisation tool
        - The audience support for PlotJuggler was incredible!

## FOSDEM 2027

FOSDEM will be back in 2027! If you're interested, you can get updates by checking the [FOSDEM website](https://fosdem.org). There will be a page for 2027 as soon as it's available, which is likely to be late in 2026.
