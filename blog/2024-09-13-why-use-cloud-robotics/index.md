---
title: "Why would I connect my robots to the cloud?"
slug: why-use-cloud-robotics
authors: mike
tags: [robotics, cloud, discussion]
---

This is a crucial question when deciding whether to use the cloud at all for robotics development, let alone how much to use it. Why connect my robots to the cloud at all? What benefits does it bring me, and what trade-offs am I making in order to use the cloud? This post is all about the *why*, rather than the *how*.

If you want the short answer, here it is: you should use the cloud if you ever plan to scale your robot fleet past around 10 robots, and if you do intend to scale up, integrate with the cloud *as early as possible* to avoid integration pains. If you're never intending to scale that much, the cloud can still be of use, but it depends on your use case as to whether the benefits outweigh the costs.

If you're not convinced, read on! The rest of this post is explaining my statement above.

<!-- truncate -->

## What is Cloud Robotics?

First, a foreword on what I mean by Cloud Robotics, as this is a term used by different people to mean different things.

In this post, Cloud Robotics means the use of cloud technologies in any part of robotics development. It may mean robots offload computation to servers in the cloud, or it could mean machine learning for a model that is deployed to a robot. It could even mean co-ordinating a fleet of robots using applications that are running in the cloud. As soon as a cloud provider is involved somewhere in development or in production, it's Cloud Robotics.

## Benefits and Drawbacks

I'll use the rest of the post to discuss these points in detail, but the main benefits and drawbacks of using Cloud Robotics are as follows:

### Benefits

1. [Scaling Up](#scaling-up): scaling software complexity, fleet size, or number of locations is difficult to do without a central system, which is a great fit for the cloud.
1. [High Power Computing](#high-power-computing): Machine Learning, GenAI, Simulation - all have high demands for computational power that are hard to meet with personal computers. The cloud offers powerful servers on demand for these resource-intensive tasks.
1. [Offloading Computation](#offloading-computation): more complex robot behaviours can be achieved on simple robots by offloading the complex parts to other servers, such as those in the cloud. You can even have the full robotics stack in the cloud, with just a thin client on every robot to read sensors and move motors.

### Drawbacks

1. [New skills](#new-skills): delving into the world of cloud computing means developing a whole new skillset, which is an investment of time, effort, and money.
1. [Costs](#costs): new users of cloud technologies find it difficult to predict cloud computing costs, which is daunting in itself. However, correctly used, moving workloads to the cloud can reduce costs by only paying for resources while they're in use.
1. [Integration](#integration): integrating cloud technologies with your robotics stack is *work*. It often means taking a longer route to get to the same outcome during development, and the more mature a robotics stack, the more difficult it is to integrate cloud technologies.
1. [Connectivity](#connectivity): the availability of the cloud is a key consideration when deciding how much to use the cloud. Robots that cannot consistently access the internet should be less reliant on the cloud. Even if the internet is available, sending messages to the cloud is slightly slower than communicating locally, so the round trip time must be justified.
1. [Security](#security): sending messages between robot and cloud means exposing those messages to the internet. There is inherently more risk with this process, but can still be secure if security is factored in to the design.

## Benefits Discussion

For each benefit and drawback, let's take a look in more detail.

### Scaling Up

The *major* benefit that the cloud brings is the ability to *scale up*. For more complex software, the size of the fleet, or the number of locations that the robots are deployed to, using the cloud is a way to tie together and support your ability to scale. Once your robotics systems are using cloud technologies, the limiting factor is no longer the support systems for the robots, but the number of robots.

This is the crucial part of using the cloud for robotics, so I'll go into depth with a few use cases in particular, although many more cases exist. For each use case, consider the difficulty of implementing the feature for 10-20 robots versus 100-200 or even 10,000+ robots:

1. **Logging and Data Storage**: the ability to stream, record, and store data and logs from a robot is vital for debugging and monitoring. 
    1. **Without the cloud**, this means increased storage capacity per robot and a server cluster that must be expanded with the number of robots and number of users that need to access the data.
    1. **With the cloud**, this process is much simpler, with services such as [Amazon S3](https://aws.amazon.com/s3/) and [Amazon CloudWatch](https://aws.amazon.com/cloudwatch/) offering to store and provide access to users with fine-grained permissions in a way that will scale with the number of robots automatically.
    1. **One resource** is [this video](https://youtu.be/cCZwQfaE6Jc), showing how to store and access logs in CloudWatch from ROS2 robots.

1. **Monitoring & Co-ordination**: monitoring and co-ordinating gets more and more complicated as the number of robots grows. 
    1. **With the cloud**, enabling auto-scaling to more servers or more powerful servers is made simple (see [Amazon EC2](https://aws.amazon.com/ec2/)), and it's possible to use serverless technologies to make parallel execution easier to manage (see [AWS Lambda](https://aws.amazon.com/lambda/) and [AWS Step Functions](https://aws.amazon.com/step-functions/)).
    1. **Without the cloud**, this is a much more difficult engineering problem, as you will need to design solutions that can be scaled up, then monitor server use to determine when to scale to more servers.
    1. **Some resources** include [running simulations on EC2](/blog/simulation-in-cloud), [co-ordinating robot fleets with Lambda](/blog/coordinating-with-lambda), and [ordering smoothies with Step Functions](/blog/step-function-make-smoothies).

1. **Deployment and OTA Updates**: setting up new robots to become part of the fleet and updating existing robots Over-The-Air (OTA) is a necessary part of scaling up. A manual update process becomes increasingly impractical as the number of robots grows. 
    1. **With the cloud**, services such as [AWS IoT Greengrass](https://aws.amazon.com/greengrass/) can make this setup and update process secure and automatic, with extra features available such as rolling back failed updates and storing per-robot configuration centrally.
    1. **Without the cloud**, you will need to build OTA updates and initial deployment process from scratch, making them as lean as possible so that adding new robots to the fleet is simple.
    1. **Some resources** include [Greengrass Concepts and Components](https://youtu.be/2VXIILtiMiU) and [deploying Docker Compose in Greengrass](/blog/docker-compose-in-gg).

1. **Dashboards**: Once data from the robots is in the cloud, it becomes simple to manage and gain insights from that data.
    1. **With the cloud**, services such as [AWS IoT SiteWise](https://aws.amazon.com/iot-sitewise/) allow users to build dashboards from their industrial data, allowing a view at-a-glance into fleet health and operational capacity.
    1. **Without the cloud**, these dashboards must be built manually along with the controls to access data from the entire fleet in one location.
    1. **Some resources** include [building a fleet overview with IoT Fleet Indexing](/blog/robot-fleet-overview), and [this video](https://youtu.be/68G4wrLXeq4) showing how to set up SiteWise to show battery measurements from a fleet of robots.

Once the cloud is used for these tasks, it is much simpler to scale the number and complexity of robots. Simply put, if you're developing in a lab setting, the cloud *can* help. If you're putting robots into production, using the cloud is the *best* way to work.

### High Power Computing

Some tasks have very high computational requirements, such as Machine Learning (ML), Generative AI (GenAI), and simulation. Each of these have such high requirements that performing them on a personal computer would be too slow to be useful. These high performance computers are available in the cloud, on demand.

- For machine learning, the cloud can store huge data sets and use powerful machines to crunch through the data to train ML models.
- For GenAI, the cloud can refine an existing Foundational Model (FM), and is capable of performing inference using an FM. FMs are very large networks, so performing inference locally is difficult. GenAI inference commonly requires multiple graphics cards working together to compute in reasonable time.
- For simulation, the cloud can offer virtual machines with a large memory capacity and high-power processors for performing simulations. Depending on the simulation complexity, it can be performed on a personal computer with high enough system specs, but the cloud makes it easier to get hold of the right hardware for running the simulation at an effective speed.

Overall, the cloud can offer powerful machines on demand for performing difficult tasks. This is particularly well-suited for bursty workloads; all of the use cases here require high compute for short periods before turning off again.

### Offloading Computation

Robots, particularly mobile robots, are limited in how much compute they can bring to bear on a problem. For more intense workloads, such as building up a map from sensor data, there are two options:

1. Upgrade the robot with more powerful hardware
1. Send the required data to more powerful hardware, then get the result back

However, there are a few reasons why you might not want to upgrade the hardware on the robot:

1. You are unable to upgrade the hardware. You have the best hardware available, or don't have power for more compute, or wouldn't be able to disperse the extra heat.
1. You have to stick to a budget, either for the current development, or a per-robot budget to keep future designs affordable.
1. You don't want to have to support more complex hardware or multiple compute devices.

In this case, your best option is to offload the computation to another server. You could deploy a server on-site to manage the computation, or you could use the pre-built servers available in the cloud, on demand. This is the kind of use case where the cloud shines - servers in the cloud are always available and ready to work, and you only pay for what you use.

## Drawbacks Discussion

The drawbacks of using the cloud in robotics are in the time and effort to learn the skills, the cost of building in the cloud, and in the security and connectivity considerations when designing the cloud into your system.

**Integration** is also hugely important to consider up front. Due to how deeply nested into the system cloud technologies can be, integrating them later in development can cause a lot of integration pain. If you decide that cloud technologies are a good fit for your stack, it makes sense to integrate them early, even though it feels like an overcomplicated solution at the time.

### New Skills

As with any new technology, there is always a cost in introducing it to your system. That might be in hiring developers with the right skills or learning the skills in the current development team. Cloud robotics is an emerging field, which makes this even more important to consider. Expect slow development while your engineers are getting up to speed with cloud technologies.

As the field develops, more learning resources and cloud services will become available to support it, meaning that the benefits will increase and the drawbacks will decrease.

### Costs

Cloud services do cost money and it can be difficult to predict those costs in advance, especially for the inexperienced. The key is to use any available tools to predict costs accurately, and adjust use over time: one of the benefits of the cloud in general is the ability to adjust workload up or down at a moment's notice. If costs are going up faster than you would like, you can turn off any non-critical systems in seconds.

However, though this is a drawback for the inexperienced, it is arguably a benefit as well. Another of the general benefits of the cloud is paying less overall, as you only pay for the services and compute that you use. Cloud providers frequently claim that moving your workload to the cloud is cheaper overall. 

### Integration

Just as there is a cost to learning cloud technologies, there is also a cost in integrating cloud technologies into your system.

Early in development, integrating the cloud could result in a more complex method of achieving the same outcome. For example, sending a message to another machine on the same network is straightforward, but if scalability is your goal, you might prefer routing that message through the cloud to get to the other machine. During development, this means extra setup and increased round trip time to send messages between two machines, but once your robots get out into the field, you already know the messaging works reliably and can be extended to other applications in the cloud.

The issue is that using the cloud feels overcomplicated early in development, but the later you leave integration, but more challenging it will be to get it working correctly. My advice is to build a proof of concept with your preferred technology, then switch to using the cloud as early as you can. This slightly increases overhead early on, but it means your systems will all be built ready to take advantage of the cloud, making the transition to production much easier.

### Connectivity

The cloud is not always accessible to robots. Some robots operate outside areas with good connectivity, such as underwater or in space; even robots in industrial facilities are not guaranteed a consistent internet connection. This is an important consideration for how much cloud to use in the system. Submarine robots should be able to operate completely independently, but may connect to the internet after returning from mission. Industrial robots may use the cloud for non-critical systems, such as uploading logs or captured data, and cache that data until an internet connection is re-established in case it drops. Only those robots with a guaranteed internet connection should rely on the cloud for their critical systems.

One extra possibility here is to deploy a server near the robots using so-called edge technologies, such as Greengrass. This is the term that the cloud uses to describe any hardware acting outside of the data centre, such as your robots, or a server you deploy on-site.

### Security

Sending and receiving data using the public internet inherently comes with risk, especially compared to a system which operates entirely within an isolated network. This is another consideration when using the cloud; sensitive data travelling unsecured could be a big problem for your stack.

Having said that, cloud providers are very focused on providing good security to their customers exactly for this reason. A cloud system that has been well-designed could end up having increased security due to the layers of encryption, authentication, identity management, and private networks that come from a full cloud setup.

When deciding whether to use the cloud with your robotics, consider how sensitive the data you are dealing with is. The more sensitive the data, the more effort you should put into ensuring good security practice - even without the cloud! This drawback does not mean you shouldn't use the cloud - even highly secure government branches use some cloud providers, given sufficient security guarantees. This drawback is more around the design effort needed to maintain a secure system.

## Summary

Overall, cloud robotics can unlock use cases that would be incredibly difficult to accomplish otherwise. Cloud providers base their businesses on highly available, secure, powerful, and scalable systems, which are exactly what's required to scale up your robotics development out of the lab. The cloud can also help support robots in the field with extra compute resources to accomplish challenging tasks, and can support development through high power compute tasks such as GenAI and Simulation.

These benefits do come at a cost. In particular, hiring in cloud experts or learning to use the technologies means time and effort with training, ambiguity with cost predictions, additional system design to ensure security, and extra effort to integrate cloud technologies into your systems - more so if the cloud is being integrated into more mature systems. The cloud is also not constantly available for all robots, which must be considered when choosing how much to rely on the cloud.

Whether the cloud is mature enough and offers enough benefits depends on your use case. My advice is that if you want to build very complex robots, or you want to scale your fleet past around 10-20 robots, the benefits outweigh the drawbacks; try to integrate the cloud as soon as possible to avoid integration and security issues. However, if you're developing smaller or simpler fleets, the cloud is not a hard requirement, but can still be helpful as a simple way to access powerful servers and large storage capacity. Take a look at some of my other blog posts for more ideas of what you can accomplish with the cloud!
