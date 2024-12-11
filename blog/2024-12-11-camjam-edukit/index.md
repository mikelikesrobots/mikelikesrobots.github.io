---
title: "Your First Robot Kit: CamJam EduKit #3"
slug: camjam-edukit
authors: mike
tags: [robotics, edukit, ros2]
---

If you want a cheap robot kit, it's hard to do better than the CamJam EduKit #3. This is a small kit which you can assemble into a two-wheeled robot with a caster, as long as you buy a few extras. It comes with a line sensor and ultrasound distance sensor, plus a series of [exercise sheets](https://github.com/CamJam-EduKit/EduKit3/tree/master/CamJam%20Edukit%203%20-%20GPIO%20Zero) to work through in order to build the robot and get it running. You can optionally get your own chassis, or you can use the box the kit comes in.

I have recorded videos that cover all of the exercises given by the repository and uploaded them to YouTube. This post goes through each video and briefly explains what it shows, so you can follow along if you choose to buy the kit. I also use the box as the chassis!

Next, I'm planning on getting ROS2 running on the robot, and using that to run collision avoidance and line following software at the same time. I will write a post on how to do that here, but if you want to see the video form, you can also [subscribe](https://www.youtube.com/@mikelikesrobots?sub_confirmation=1) to my [YouTube channel](https://www.youtube.com/@mikelikesrobots) to be notified about it.

## Parts Required

To be able to follow along, you will need to buy the robot kit and a few extra parts. You can use substitutes for any of these except for the robot kit; these links are to the parts that I used for my videos.

- [CamJam Edukit #3](https://thepihut.com/products/- camjam-edukit-3-robotics)
- A Raspberry Pi
  - [Raspberry Pi Zero 2 W](https://thepihut.com/products/raspberry-pi-zero-2) OR
  - [Raspberry Pi 4](https://www.amazon.co.uk/Raspberry-Pi-Model-4GB/dp/B09TTNF8BT?)
  - [Raspberry Pi 5](https://www.amazon.co.uk/Raspberry-Pi-SC1112-5-8GB/dp/B0CK3L9WD3)
- [Power Bank](https://thepihut.com/products/- ansmann-10-000mah-type-c-18w-pd-power-bank)
  - *Note: this is too large to fit in the robot! You should consider buying a slightly smaller power bank.*
- [AA batteries](https://www.amazon.co.uk/- Duracell-Plus-Alkaline-Batteries-MN1500/dp/B093LVB4P7)
- [MicroSD Card](https://www.amazon.co.uk/- SanDisk-128GB-microSDXC-adapter-Performance/dp/B0B7NTY2S6)

Once you have all the parts, you can begin assembly!

## Building the Robot

If you want to watch the whole series in order, navigate to [the playlist](https://www.youtube.com/playlist?list=PLBrq1OKRHMwUbbujTlmt1YGRzL9O0LfNJ). Otherwise, I list out all of the videos and what I show in them, so you can choose which parts you want to follow.

Each video has extra resources depending on the specific video, but all the videos require:
- [EduKit website](https://camjam.me/?page_id=1035)
- [Github repository](https://github.com/CamJam-EduKit/EduKit3)

### Introduction

This is a short introduction explaining what I'll be going over in the series. There isn't much more in the video than I've written in this post.

<iframe class="youtube-video" src="https://www.youtube.com/embed/dH0sKmp1DMw?si=G50w2s7UHqu79fkF" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Initial Build

This video is a top-down view of assembling the robot from its contents. I don't attach the sensors yet - this is to get the batteries, motors, and Raspberry Pi ready to roll. After this, the robot is physically ready to begin moving around, and the next step is setting up the software.

<iframe class="youtube-video" src="https://www.youtube.com/embed/q2bKTyTTquU?si=lHksN2IWe81UJbdx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### SD Card Setup

This video shows how to set up the SD card ready to move the motors, with one key difference: I install Ubuntu instead of Debian to make it easier to install ROS2 down the road.

Some extra resources for this video are as follows:

- [Raspberry Pi Imager](https://www.raspberrypi.com/software/) - to flash the SD card
- [Create an SSH key pair](https://learn.microsoft.com/en-us/viva/glint/setup/sftp-ssh-key-gen) - to explain the SSH key generation steps
- [Visual Studio Code](https://code.visualstudio.com/download) - to download Visual Studio Code (VSCode), the editor I recommend for this course (and in general!)

<iframe class="youtube-video" src="https://www.youtube.com/embed/MzBFo65xnbA?si=x1cZMRTW8Q_fpZin" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Adding Sensors

This video is the follow up to the [initial build](#initial-build), where I show how to add both sensors to the robot.

<iframe class="youtube-video" src="https://www.youtube.com/embed/oPJ07Bl87Ts?si=hIJ9flpp0muP8JX9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Testing the Line Sensor

I show how to test and adjust the line sensor to get a reliable reading on the robot.

<iframe class="youtube-video" src="https://www.youtube.com/embed/1jSZvPHQuDo?si=17B0eTIS7gtn4DYm" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Testing the Distance Sensor

I show how to test the ultrasound sensor to get a distance reading from the robot.

<iframe class="youtube-video" src="https://www.youtube.com/embed/WDr2CTLzRRg?si=aTFBnU5qWztyyD78" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### First Robot Movement

At last, we can move the robot around for the first time! I also show moving the weight to make sure the wheels get more grip, as before that, it mostly just spins in place.

<iframe class="youtube-video" src="https://www.youtube.com/embed/ZzPUg-0znsI?si=9nzTiLR7aWEXuo9Z" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Line Following Course

Having moved the robot for the first time, I show the script provided to follow a line course and explain why the robot is spinning from side to side so much while following.

<iframe class="youtube-video" src="https://www.youtube.com/embed/0NGg4-DZkBA?si=KHpJUi-CYtXXdgAy" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Obstacle Avoidance

This is the last video based on the exercises provided by CamJam. In it, I show how to get the robot avoiding collisions with objects in the view of the sensor, again using a provided script.

<iframe class="youtube-video" src="https://www.youtube.com/embed/CSdSrhLD8iQ?si=vM7m1NXKVDkYFiz4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Next Steps

Having finished the exercise sheets, I plan to install ROS2 on to the board. I need to experiment to figure out if directly compiling ROS2 or installing via Docker has the best performance. Once ROS2 is installed, I'll show how to set up a project to:
- move the robot around
- follow a line
- avoid obstacles
- to accomplish all of these at the same time

It is also possible I could add more sensors to the robot in future, such as another line sensor or wheel encoder. If you're interested, please leave a comment here or on my YouTube videos.
