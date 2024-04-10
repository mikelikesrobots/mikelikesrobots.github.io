---
title: "Making Smoothies with AWS Step Functions and Robots!"
slug: step-function-make-smoothies
authors: mike
tags: [aws, robotics, communication, lambda, stepfunctions]
---

This post is about how to build an AWS Step Functions state machine, plus how you can use it to interact with IoT edge devices - in this case, sending a smoothie order to a "robot" and waiting for it to make that smoothie.

It works by using a series of Lambda functions, and defining how data should be passed between them. You can read more about using Lambda functions in robotics in [this blog article](/blog/coordinating-with-lambda). There's also a step where the state machine needs to wait for the drink to be made, which is slightly more complicated - we'll cover that later in this post!

This post is also available in video form - check the video link below if you want to follow along!

<!-- Lambda post showed how to create a DDB table with CDK and update it using Lambda functions.
This post is about combining multiple Lambda functions into a serverless cloud application.
It also uses IoT rules - see video. -->

## What are Step Functions?

[AWS Step Functions](https://aws.amazon.com/step-functions/) is a way of building serverless workflows. Serverless came up in [my post on Lambda functions](/blog/coordinating-with-lambda) - it means that you can run applications in the cloud without provisioning any servers or contantly-running resources. That in turns means you only pay for the time that something is executing in the cloud, which is often much cheaper than provisioning a server - but with the same performance.

In this case, we're assuming that a customer will order a smoothie, and we need to send that order to one of our available robots, then wait for the robot to finish making the order. To handle this smoothie order, we have a Step Functions state machine in the cloud that we can access using [the console](https://us-west-2.console.aws.amazon.com/states/home?region=us-west-2#/statemachines). Click the Edit button to open up the workflow Design tab for a visual representation of the state machine:

![Visual representation of Step Functions State Machine](./img/edit-workflow.webp)



First, we'll look at the overall state diagram. Here it is.
What's not in this diagram is the "robot", which communicates using MQTT. We will publish a message to a robot to request a smoothie, wait for the robot to make the smoothie, then it will publish a message to say it is finished with the smoothie. We use an IoT Rule to listen to that message and call another Lambda function.

Let's look at its parts. We have the state machine, which points to different Lambdas. We define how data is moved from piece to piece. We could time out making the drink, so we have the happy path and the error path. Here's the IoT rule. Here's the DDB table.

Let's see it working. We run the test script.. Run the drink order.. Take a look at the execution. Look at DDB contents. It's working, then it's online again and ready for more orders.
Let's see the unhappy path - a robot had a malfunction and never completed the drink, or accepted the order. We can just stop the mock robot script, request a smoothie, and look - our execution fails; our robot is set to broken. If we run it again, it will now select robot2 as the first instance.

If we don't have any robots available, that's that. GG.

So how do we assemble this? Let's look at some code.

Central CDK stack. This calls out to a few components, to divide things up a little.

1. DDB table - simple, contains robots and their statuses
2. 4 Lambda functions - we'll get to those after the CDK
3. IoT Rule - listens for success, calls a lambda to alert the step function to continue execution
4. Step Functions - this is where we define the execution order. We've got the Lambda functions already.

Okay. So we've seen what each of the tasks do. Let's take a look at the handlers.

Each follows the same basic pattern that we saw in the last Lambda article. The update status function has two more statuses enabled. Otherwise, the functions are simple:
1. Send an MQTT message with the drink order to a topic specific to the robot
2. Get the first available robot
3. Alert the step function that the task is done

That's it! Build... Deploy... Run the get script... Now you can run mock robot... Run the step function test script... You can do it all yourself.

Challenge: make the retry branch go back to the start of the state machine.
Another challenge: feed messages in via SQS so customer's orders can be backed up into a queue.

<!-- Still need to update the README with setup instructions -->
