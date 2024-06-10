---
title: "How to Build a Fleet Overview with AWS IoT Fleet Indexing"
slug: robot-fleet-overview
authors: mike
tags: [robotics]
---

Congratulations! You have a whole lab full of robots running your latest software. Now you want to start looking at an **overall** view of your robots. It's time to build a Fleet Overview, and this post will show you how to do that using Fleet Indexing from AWS IoT Core.

Fleet Indexing is a feature of AWS IoT Core that collects and indexes information about all of your selected Things and allows you to execute queries on them and aggregate data about them. For example, you can check which of your Things are online, giving you an easy way to determine which of your robots are connected.

I want to walk you through this process and show you what it looks like in the console and using Command Line Interface (CLI) commands. We'll be using the sample code from TODO, but I've forked it to add some helper scripts that make setup a little easier for multiple robots. It also adds a launch script so multiple robots can be launched at the same time.

## Fleet Indexing

<!-- Talk more about fleet indexing. What is it? -->
<!-- TODO talk more about Device Management? -->

[Fleet Indexing](https://docs.aws.amazon.com/iot/latest/developerguide/iot-indexing.html) is a feature of IoT Device Management that allows you to index, search, and aggregate your device data from multiple AWS IoT sources. Once the index has been enabled and built, you would be able to run queries such as:

- How many devices do I have online?
- How many devices have less than 30% battery left?
- Which devices are currently on a mission?
- What is the average metres travelled for my fleet of robots?

To perform these queries, AWS has a query language which can be used for console or CLI commands. There is also the option to aggregate data over time, post that aggregated data to CloudWatch (allowing for building dashboards), and enabling alarms on the aggregate state of your fleet based on pre-defined thresholds.

Device Management has a suite of features, including bulk registration, device jobs (allowing for OTA updates, for example), and secure tunnelling. I won't go into depth on these - Fleet Indexing is the focus of this post.

### Pricing

The [Device Management pricing page](https://aws.amazon.com/iot-device-management/pricing/) has the most detail. There is a very low charge for registering devices, then charges to update the index.

Fleet Indexing is opt-in. Every service added to the index, such as connectivity or shadow indexing, will increase the size of the index and what operations will update the index. For example, if connectivity is not enabled, then a device coming online or offline will not update the index, so there will be no additional fleet indexing charge. If shadow indexing is enabled, then every shadow update will incur a charge, measured in a few USD per million updates at time of writing.

There is also a charge of a few cents per thousand queries.

Overall, we can keep this in mind when deciding which features to include in the Fleet Index given a particular budget. Frequent shadow updates from a large robot fleet will have a larger associated cost.

TODO: AWS pricing video

## Setup Guide



Let's start by looking at the console. Take a look, here's the info page and some documentation. Now let's enable it for our fleet, click here, click here, done.

Let's show it in action. To show you some sample data, I'm going to use the safe cracker robot from the aws samples repository, which is used to demonstrate the use of an IoT Shadow for passing data.

I'll show you how I set it up. AWS CLI with creds; install deps; run script; build workspace; launch system. Now the shadow is being updated. I'll show the shadows, here they are updating, great. Now we have sample data.

Now we can start to see fleet indexing in action. Without any extra information, we'll see what we get.

1. Show Things (console + CLI)
1. Show connectivity (console + CLI)
1. Show shadows (console + CLI)

Let's show it in action. For this, I'll show you one part at a time what gets added. We start by enabling fleet indexing, then checking out our new capabilities. Search, look for robot*. We can do the equivalent on the command line: `do that`


