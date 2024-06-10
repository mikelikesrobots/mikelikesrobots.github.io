---
title: "How to Build a Fleet Overview with AWS IoT Fleet Indexing"
slug: robot-fleet-overview
authors: mike
tags: [robotics]
---

Incredible! You've built a whole fleet of robots and you have them operating in your lab. Now, you want to level up - how can you start looking at your fleet as a whole, instead of your robots as individuals?



Let's say you have a fleet of robots, in various locations, and you want to get an overview of the fleet. How are you going to do that?
Well, you could manually update the shadows with the statuses yourself. Or you could build rules that update a DynamoDB table. But I have another solution for you: Fleet Indexing.

This is a feature of AWS IoT Core that collects and indexes information about all of your selected Things and allows you to perform queries and aggregate data about them. For example, you can use IoT Connectivity to check which of your Things are online, meaning an easy way to determine which of your robots are connected!

I want to walk you through this process and show you what it looks like in the console and using CLI commands. We'll be using the sample code from TODO, but I've forked it to add some helper scripts that make setup a little easier for multiple robots. It also adds a launch script so multiple robots can be launched at the same time.

Let's start by looking at the console. Take a look, here's the info page and some documentation. Now let's enable it for our fleet, click here, click here, done.

Let's show it in action. To show you some sample data, I'm going to use the safe cracker robot from the aws samples repository, which is used to demonstrate the use of an IoT Shadow for passing data.

I'll show you how I set it up. AWS CLI with creds; install deps; run script; build workspace; launch system. Now the shadow is being updated. I'll show the shadows, here they are updating, great. Now we have sample data.

Now we can start to see fleet indexing in action. Without any extra information, we'll see what we get.

1. Show Things (console + CLI)
1. Show connectivity (console + CLI)
1. Show shadows (console + CLI)

Let's show it in action. For this, I'll show you one part at a time what gets added. We start by enabling fleet indexing, then checking out our new capabilities. Search, look for robot*. We can do the equivalent on the command line: `do that`


