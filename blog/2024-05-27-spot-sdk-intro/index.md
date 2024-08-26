---
title: "How to use the Boston Dynamics Spot SDK"
slug: spot-sdk-intro
authors: mike
tags: [robotics, educational, spot]
---

This post is to show how to set up the Boston Dynamics Spot SDK.

If you prefer a video format, or you want to see the samples in action, check out my YouTube video below:

<iframe class="youtube-video" src="https://www.youtube.com/embed/Hsa1wPpJA_Q?si=WXyIVy1k1_gGd0V0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<!-- truncate -->

## Boston Dynamics Spot SDK

Boston Dynamics released the Spot robot, and I was able to get my hands on one in the lab. I also released the video below to show the basics of getting it unpacked and moving around.

<iframe class="youtube-video" src="https://www.youtube.com/embed/lNwuCiZdzQE?si=xi9WZHcZkkRjIUI1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

This post is about showing examples from the Boston Dynamics Spot SDK, including mapping its environment and moving autonomously to key waypoints in the map, or detecting and following a person around. Unfortunately, it's hard to demonstrate the samples working in this post! I'll talk about the setup of the SDK and using it to connect to the robot, then leave it to the video for showing the samples themselves.

## Downloading the SDK

The SDK is available from [this Github repository](https://github.com/boston-dynamics/spot-sdk), and Boston Dynamics host the [documentation for the SDK](https://dev.bostondynamics.com/readme). Most importantly, we're interested in the [Python examples](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples).

Start by cloning the SDK in your chosen terminal using `git`:

```bash
git clone https://github.com/boston-dynamics/spot-sdk
```

Make sure you have `python` installed - if not, [follow the instructions](https://www.python.org/downloads/) for your system to install.

The next step is to set up a virtual environment for dependencies. This is entirely optional, but it makes it easier to track Python dependencies if they're all installed like this.

To set it up, make sure `pip3` is available:

```bash
pip3 --version
```

Then use it to install `virtualenv`:

```bash
pip3 install virtualenv
```

Once `virtualenv` is installed, use it to create a virtual environment and activate it:

```bash
cd spot-sdk
virtualenv venv
# On Windows:
venv\Scripts\activate
# On Mac:
source venv/bin/activate
```

Now we have a clean virtual environment, we can install any dependencies we want into it. For example, we can install dependencies for the `hello-spot` example:

```bash
pip install -r ./python/examples/hello_spot/requirements.txt
```

This same process can be used for every example that you want to run. You do need an internet connection to do this, so it's a good idea to install dependencies you're likely to need now to save reconnecting later.

## Hello Spot

To understand how to connect to the robot, the `Hello Spot` example is a great place to start. Let's take a look at how it works! The code we're looking at is the [`hello_spot.py`](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py) file. In addition, we can look up any concepts in the [Boston Dynamics Concepts documentation](https://dev.bostondynamics.com/docs/concepts/readme) - the code provides a great overview, with the documentation going into more detail about each part.

The interesting parts are all in the `hello_spot` function. I'll go through a few lines and describe what they're doing. First, a minor but useful step: setting up logging.

```python
bosdyn.client.util.setup_logging(config.verbose)
```

This [first line](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L34) is using the Python logging module, as described in the code comments. It's using the verbose argument set in config so you can change how verbose the logging is when you start the script.

The next step is to [create the standard SDK](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L39). This is the object used to interact with the SDK.

```python
sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
```

The SDK object is used to [create the robot object](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L46), which is a Python object representing the robot that we can interact with. This is where the host name is used to contact the robot, which is one of the command line parameters. Get this wrong, or be connected to the wrong network so that you can't contact the robot, and this stage will fail.

```python
robot = sdk.create_robot(config.hostname)
```

The next step is to [authenticate with the robot](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L49) using the username and password. The SDK provides a utility method which prompts the user (you) for the username and password to connect to the robot.

```python
bosdyn.client.util.authenticate(robot)
```

Once authenticated, we need to [synchronize time with the robot](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L54). This is because the robot has its own idea of the current time and will refuse commands sent with a time difference that's too large. For more information here, see the [concepts page on time-sync](https://dev.bostondynamics.com/docs/concepts/base_services.html#time-sync) in the documentation.

```python
robot.time_sync.wait_for_sync()
```

The example then [asserts that the robot](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L58-L59) is not [estopped](https://dev.bostondynamics.com/docs/concepts/estop_service) (i.e. emergency stopped). To be able to drive the robot, someone must be able to press the estop. This can be done using the tablet with the Spot app or the estop example from the SDK. If someone is able to press the estop, and the estop has not already been pressed, then execution can continue.

```python
assert not robot.is_estopped(), 'assertion message'
```

The comments explain the concept of [leasing](https://dev.bostondynamics.com/docs/concepts/lease_service) very well, so I'll include [the relevant comments](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py#L65-L72):

```python
# Only one client at a time can operate a robot. Clients acquire a lease to
# indicate that they want to control a robot. Acquiring may fail if another
# client is currently controlling the robot. When the client is done
# controlling the robot, it should return the lease so other clients can
# control it. The LeaseKeepAlive object takes care of acquiring and returning
# the lease for us.
lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
```

The rest of the code stays within the `with` block, meaning that it can access the robot and move it around. From there, it is possible to give the robot movement commands, including being able to stand up.

This is the basic pattern for all robot scripts that involve movement:

1. Create SDK object
1. Create robot object from SDK
1. Authenticate with robot
1. Synchronize time with robot
1. Check estop
1. Obtain robot lease

## Message Format and Request/Response

The way that our application sends messages to the Spot robot is by building [Protocol Buffers](https://protobuf.dev/) (aka protobuf) messages and sending them using [gRPC](https://grpc.io/). For the most part, we don't need to understand this, as the SDK does a good job of hiding the messaging behind SDK methods - but once we get to the more advanced examples, understanding how protobuf and gRPC work can help understand the code.

We can use protobuf to define the structure of a message, then generate the code for serializing and deserializing an instance of that message to bytes. The SDK provides the messages and the pre-generated methods for serializing and deserializing those messages for us.

At the same time, we use gRPC to send and receive messages. You can think of this as similar to a ROS service call, which also has a request/response flow. For the Spot SDK, there are defined services available, which we can interact with by using the relevant Request and Response messages. 

Take the `arm_joint_move` example, which has a method to [build an arm movement command](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/arm_joint_move/arm_joint_move.py#L38-L47):

```python
def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)
```

We can see in this code that we construct an `arm_command_pb2.ArmCommand.Request`, so we can reasonably expect an `arm_command_pb2.ArmCommand.Response` in response. We can also see that the SDK provides a `RobotCommandBuilder` to help us build messages for particular services.

This flow is in many places that don't have specific SDK methods. Anywhere that you see `_pb2` in the example code is generated by protobuf. Have fun reading through the samples!

## Examples

As written earlier in this post, it's difficult to show the working examples in a blog post! Please do take a look at the video if you want to see the following samples in action:

1. [`hello_spot`](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/hello_spot)
1. [`estop`](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/estop)
1. [`graph_nav_command_line`](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/graph_nav_command_line)
1. [`graph_nav_view_map`](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_view_map/README.md)
1. [`spot_detect_and_follow`](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/spot_detect_and_follow)

## Summary

In short, that's the basics of interacting with Spot using Python! From there, it's up to you what you can get the robot to do. In the future, I'm eager to get my robot connected to AWS to show how the cloud can provide value for it - maybe I can control it using a [Lambda function](/blog/coordinating-with-lambda/) in the cloud!
