---
title: "ROS2 on a Raspberry Pi Robot"
slug: raspi-camjam-ros2
authors: mike
tags: [robotics, ros2, edukit]
---

<!-- Intro, previous blog post, video -->

<!-- TODO: links -->
This post is about showing how to run ROS2 on a Raspberry Pi Zero board. The
Raspberry Pi Zero is a small development board with minimal compute power, but
it is able to run a full Linux operating system, which makes it a great entry
point into building your own cost-effective robot.

I use the board with the CamJam Edukit #3 robot kit. I have a series of videos
on YouTube of how to assemble and run the kit, and a blog post that will tell
you what each video is for. Having said that, both the robot kit and the
instructions in this post are just as applicable for a more powerful Raspberry
Pi, such as the 4 or 5 models.

This post is also available in video form. If you'd prefer to watch, click the
link below:

TODO

<!-- truncate -->

## Why use ROS2?

<!-- TODO link full blog post -->
This is a large enough question that I decided to write a full blog post about
it.

<!-- TODO add Jetbot link -->
The short answer is that ROS2 isn't *that* useful for a robot of this
complexity, but it paves the way for us to add more complexity in, and it does
have some useful benefits. For example, we could plug a gamepad into a computer
on the same network as the Pi and control it using ROS2, if we set it up right.
You can see how I accomplish this on the JetBot.

In this case, we will use it to continuously **publish** the sensor data from
both the distance sensor and the line sensor, then have separate **nodes** to
determine how to move the robot and to translate movement commands into moving
the wheels.

By using this setup, we could either use the sensors to decide how to move the
robot autonomously, or we can directly send movement commands from a gamepad to
control the robot manually. We can add more sensors in more easily, and we can
reuse the code across other robots if we want.

With that in mind, let's take a look at how to set up ROS2 on the Raspberry Pi
Zero.

## Setting up ROS2

<!-- TODO link to install from source -->
The easiest way to set up ROS2 on Ubuntu is to install the binaries using apt.
If you're not using Ubuntu, you may need to install from source. The reason I
used Ubuntu for my Raspberry Pi so far is to make it easier to install ROS2 now,
so if you've been following along, it should be easy!

:::warning

**Try not to use VSCode over SSH for these instructions**.

In the past, I used VSCode over SSH to edit and run scripts on the robot. For
installing ROS2, I recommend using a terminal to SSH instead. The board is not
very powerful, so running any ROS2 command has a high chance of freezing and
restarting the board - at least from my own testing.

:::

To install ROS2, you can follow the instructions for your board depending on the
version of Ubuntu you are running:

- Ubuntu Jammy (22.04): ROS2 Humble or Iron
- Ubuntu Noble (24.04): ROS2 Jazzy

I reflashed my own board to use Ubuntu Noble, so I use the [installation
instructions for ROS2
Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### SSH Connection

Open a terminal, such as Command Prompt on Windows or Terminal on Linux. If you
are using Linux or Mac, you should have SSH already available. If you are on
Windows, try to install [OpenSSH following these
instructions](https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse).

Once complete, make sure your computer and the Raspberry Pi are both on and
connected to the same network. SSH in to the board using `ssh user@rpi0` - these
will depend on the username and hostname that you set while flashing the board.
For me, this is `ssh mike@rpi0`.

All commands from here on are run in the SSH session with the Raspberry Pi. If
the board goes down, keep trying to SSH in again until it accepts the
connection.

### System Setup

Run `locale` and check all of the entries contain `UTF-8`.

To enable the required repositories, execute the following commands:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS2 GPG key:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository for ROS2. We added the GPG key so that the board will
connect to this repository without errors.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

With all the system dependencies installed, it's time to install ROS2 itself.

### Installing ROS2

First, make sure the system is up to date after adding the new repositories.
This may take some time if you've not done it after flashing:

```bash
sudo apt update && sudo apt upgrade -y
```

For this tutorial, we will need the right ROS2 version and the dev tools.
Replace `jazzy` with your version of ROS2 for these instructions.

```bash
sudo apt update && sudo apt install ros-dev-tools ros-jazzy-desktop
```

Again, this is likely to take some time. Once complete, you should be able to
run the following command:

```bash
source /opt/ros/jazzy/setup.bash
```

If this works, you're in good shape to start checking out the code! If not, make
sure you installed the right packages above, and take a look at the installation
instructions to see if you (or I) missed anything.

:::note

Running this `source` command is important to be able to run any ROS2 command.
If you open a new SSH session, or the board restarts for any reason, make sure
to run this command again.

:::

With ROS2 installed and ready to go, let's take a look at how to download and
build the source code.

## Building the CamJam Packages

### Creating a Workspace

ROS2 uses workspaces to keep track of everything it needs for a robotics
application. You can use any folder you want as a workspace, but I tend to use
`ros2_ws` as the workspace name, where `ws` stands for workspace.

If you want to use another name, feel free to edit the commands below.

First, create the workspace folders and change directory into the `src` folder.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Then, clone the packages for the CamJam into the `src` folder. Each package
contains one or more nodes which are useful for running the robot. By cloning
each one, we can build everything together in the same workspace and be able to
launch it all at the same time.



## Running the Robot Software with ROS2

## Summary
