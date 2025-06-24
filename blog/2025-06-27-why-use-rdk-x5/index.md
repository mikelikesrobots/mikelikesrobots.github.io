---
title: "WHy You Should Use the RDK X5"
slug: why-use-rdk-x5
authors: mike
tags: [robotics, rdk, x5, ai]
---

:::warning Sponsored Content

This article was sponsored by D-Robotics. However, my thoughts and opinions are my own.

:::

## The RDK X5

<!-- TODO: link -->
You may remember my exploration of AI on the RDK X3. I took a look at different models than can run locally on board, and discussed the advantages of being able to run AI locally instead of using the cloud or an edge server.

The RDK X5 is the successor to the RDK X3, and it's being launched _right now_. This board is similar to the X3, but with a more powerful CPU and BPU. The BPU is the Brain Processing Unit, which is what D-Robotics calls a Neural Processing Unit, and it's what makes the board special - the chip can run AI models on board at incredible speeds. D-Robotics states that the chip has 10 TOPS of power, or Tera Operations per Second. For comparison, an NVIDIA mobile GPU can deliver 1000 TOPS.

RDK stands for Robot Development Kit. The purpose of the board is to be used to build intelligent robots, and it provides the hardware and technical support to do so. The documentation, the technical community in the discord and forums - they serve to help developers fix issues they have when building robots with the RDK.

The three main reasons to use an RDK X5 for your next robotics project are:

1. Ease of setup
1. Peripherals Available
1. Performance
1. Price

I'll elaborate on each point in the next sections.

## Ease of Setup

The board has a familiar setup process to other boards in its class. All that's needed is to download the firmware image from D-Robotics and flash it onto an SD card. The board boots from the SD card and immediately has ROS 2 installed, with some AI models ready to run using sample applications.

The board has useful peripherals, like a full-size HDMI port, although I always use headless setups with boards I intend to use for robotics. Still, the option is there! You can use a monitor and keyboard for first-time setup, or connect using serial cable, USB, or ethernet cable. Once you've connected, you can join a WiFi network and connect through that as well.

Overall, the process is incredibly simple. Flash the SD card, boot the board, plug an ethernet cable in to set up WiFi, then connect using WiFi and set an example application going. 

## Peripherals Available

D-Robotics sells a number of peripherals that you can connect to the board to get more out of it. For a start, MIPI cameras, USB cameras, and MIPI stereo depth cameras are all supported - this last one is two RGB cameras with an AI model predicting distances.

You can also plug in a microphone to the aux port, and standard USB devices are supported. Anything that can run on Ubuntu will work with the RDK. This includes the 2D lidar I have on my OriginBot.

<!-- Show the peripherals! Get some images on there! -->

## Performance

The X5 offers equivalent or better performance to other boards of its class with CPU alone. The additional BPU means that it can do more! It can run YOLO at 200fps using hardware acceleration, so you can have your normal robot software running on the CPU with the AI running on the BPU. Between the two, you can start to get much more intelligent applications.

<!-- TODO: link X3 article -->
<!-- TODO: check YOLO runtime stats -->
You can take a look at my article on AI with the RDK X3. Note that performance for the X5 should be better due to the improved hardware specs. The company states that YOLO X runs at 200fps.

## Price

It only costs $x to buy, isn't that cool

## Conclusion
