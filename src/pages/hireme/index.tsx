import Heading from "@theme/Heading";
import Layout from "@theme/Layout";
import clsx from "clsx";
import styles from "./index.module.css";
import Link from "@docusaurus/Link";
import {
  CapabilityCard,
  CapabilityCardProps,
} from "@site/src/components/CapabilityCard";
import {
  CustomerTestimonial,
  CustomerTestimonialProps,
} from "@site/src/components/CustomerTestimonial";
import {
  PastProject,
  PastProjectProps,
} from "@site/src/components/PastProject";

const CapabilityCardDefs: CapabilityCardProps[] = [
  {
    title: "IoT & Embedded Devices",
    description: (
      <>
        I can design embedded systems with optional cloud connectivity, using
        chips such as <b>ESP32</b> and <b>STM32</b>, among others. My experience
        includes PCB design, sensor integration, and working with <b>UART</b>,{" "}
        <b>SPI</b>, and <b>I2C</b> protocols. I always ensure reliable, scalable
        solutions, whether built from scratch or reviewing an existing design.
      </>
    ),
  },
  {
    title: "Robotics & Automation",
    description: (
      <>
        Robots are my speciality and my passion. I can work with you to design
        or improve an existing robot, all the way from <b>embedded systems</b>{" "}
        to <b>cloud integration</b>. As I have been using <b>ROS</b> and{" "}
        <b>ROS 2</b> for my entire career, I can help you build controllers,
        write localization and navigation systems, and make your robots behave
        more intelligently with <b>Artificial Intelligence (AI)</b> solutions.
      </>
    ),
  },
  {
    title: "Cloud Infrastructure",
    description: (
      <>
        Robots cannot solve problems by themselves; they require{" "}
        <b>reliable infrastructure</b> to do their jobs. I can work with you to
        build the tools your robots need to solve the problems they were built
        for. I am able to use <b>cloud</b> or <b>edge</b> systems, deployed
        manually or via <b>Infrastructure-as-Code (IaC)</b>, to ensure{" "}
        <b>seamless communication</b>, <b>remote deployment</b>, and efficient{" "}
        <b>fleet management</b>.
      </>
    ),
  },
];

const TestimonialDefs: CustomerTestimonialProps[] = [
  {
    testimonial: `Mike has been a huge help getting our project moving forward and
       stabilized. He's fit right into our team and got right to work with
       helping us with our IoT project. We are happy with his work and thought
       process. Plan to continue working with Mike for the foreseeable future!`,
    customerName: "Nicholas Aron",
    position: "CEO",
    company: "Aron Corp - CentralFlo",
  },
];

const PastProjectDefs: PastProjectProps[] = [
  {
    title: "IoT Measurement and Control Platform",
    client: "Aron Corp - CentralFlo",
    objective: `Develop software for custom ESP32 board to enable sensor reading, remote
       command and control, and Over-The-Air (OTA) updates.`,
    details: [
      `Proposed and implemented move from Arduino to FreeRTOS system for
       a multi-threaded, more reliable, and more secure firmware.`,
      `Built OTA update mechanism to allow software updates to
       be deployed from AWS to groups of devices, according to a rollout plan.`,
      `Proposed and created schema for bi-directional communication between
       edge devices and cloud-based control platform.`,
    ],
    outcome: `Devices went from an early development stage through to beta
      testing with customers. The team verified that device operation was much
      faster and more reliable than the initial design.`,
    image: "/img/hire_icons/iot_platform.webp",
    alt: "Abstract icon representing IoT devices talking to cloud-based IoT platform",
  },
  {
    title: "Amazon Scout Delivery Rover Performance Improvements",
    client: "Amazon.com Inc (As Employee)",
    objective: `Optimise existing autonomous ROS-based mobile robot for delivering
      packages to customers.`,
    details: [
      `Proposed and led team of 5 engineers to build a performance profiling and
       simulation framework for replaying recorded ROS data.`,
      `Tools generated detailed mission reports, allowing deep dives and
       performance improvements.`,
      `Identified and optimised specific pieces of code, such as rewriting a
       slow node from Python to C++.`,
    ],
    outcome: `The tools built served as a basis for hardware-in-the-loop (HIL)
       testing, and led to performance improvements across the board.
       Unfortunately, these were hard to measure as the software on the robot
       was becoming increasingly complex in other areas.`,
    image: "/img/hire_icons/amazonscout-3.webp",
    alt: "Image showing Amazon Scout from the side.",
  },
  {
    title: "Machine Learning (ML) for Warehouse Robot Perception",
    client: "AWS (As Employee)",
    objective: `Develop new AWS service for providing ML perception at the edge to
      warehouse robots. The model improved over time as more data was
      collected.`,
    details: [
      `Led the team to build the prototype version of the service.`,
      `Built deployment mechanisms for performing perception at the edge,
       including identifying suitable edge hardware.`,
      `Agreed to test proof-of-concept with large robot arm manufacturer.`,
    ],
    outcome: `Despite shipping the first prototype capable of performing ML-based
       perception at the edge to a partner, the service was not deemed
       profitable enough to continue, and the project was cancelled.`,
    image: "/img/hire_icons/robin_arm.webp",
    alt: "Image showing robot arm from Amazon Warehouse.",
  },
  {
    title: "Room-Tidying Robot Arm Concept",
    client: "Cambridge Consultants (As Employee)",
    objective: `Build robot arm capable of picking up objects from a messy room and
       placing in a box. (Picture is not the same robot.)`,
    details: [
      `Created custom software for messaging between the robot arm and the
       customer interface using gRPC.`,
      `Captured data from VR environment to train ML model on how to pick up
       difficult objects, such as mugs or cuddly toys.`,
      `Built highly accurate calibration system for the robot arm and stereo
       cameras used to detect objects in the scene.`,
    ],
    outcome: `The robot was successfully demonstrated at live events hosted by
       the company, and was used in online marketing material.`,
    image: "/img/hire_icons/room_tidying.webp",
    alt: "Image showing robot arm with objects to be tidied away.",
  },
];

export default function HireMe(): JSX.Element {
  return (
    <Layout
      title={`Hire Me`}
      description="How I can help you with your robotics project"
    >
      <div className={clsx("hero hero--primary", styles.heroBanner)}>
        <div className={clsx(styles.collapsiblePadding)}></div>
        <div className={clsx(styles.flexColumn)}>
          <Heading as="h1" className={clsx("hero__title")}>
            Robotics & IoT Software
          </Heading>
          <p className={clsx("hero__subtitle")}>
            I can help you build custom software for smart robots and IoT
            systems.
          </p>
        </div>
      </div>

      <main>
        <section className={clsx("container padding-vert--md")}>
          <div className={clsx(styles.descriptionBox)}>
            <h3>About Me</h3>
            <p>
              I am an educator, leader of the{" "}
              <a href="https://cloudroboticshub.github.io/">
                Cloud Robotics Working Group
              </a>
              , and a specialist in writing software for{" "}
              <a href="/blog/why-use-cloud-robotics">
                connecting robots to the cloud
              </a>
              . I help my clients to bring their robot ideas to life or improve
              their existing designs. My experience is primarily in research &
              development and prototyping, helping clients build and refine
              their robotic systems. However, while my focus is on early-stage
              development, I also have the skills to support production
              workloads.
            </p>
            <p>
              Over my 12 year career, I have worked as a consultant and as a
              senior SDE at AWS, all with a focus on robotics. This required me
              to work on various projects (including non-robotics projects),
              meaning that I am able to work with many different technologies -
              those required to build robot software and those required to
              support them. Hence, I am able to work with electronic and
              embedded systems, all the way through to leveraging the cloud.
            </p>
            <p>
              To find out more, you can read more below, take a look at my{" "}
              <a href="/blog">blog</a> and&nbsp;
              <a href="https://youtube.com/@mikelikesrobots">YouTube channel</a>
              , or contact me using one of the following methods:
            </p>

            <div
              className={clsx(
                "text--center padding-vert--xs",
                styles.linkButtons
              )}
            >
              <Link
                className="button button--secondary button--md item shadow--md"
                to="https://www.linkedin.com/in/michael-hart-a7614262/"
              >
                <i className="fab fa-linkedin fa-lg padding-right--sm"></i>
                LinkedIn
              </Link>
              <Link
                className="button button--secondary button--md item shadow--md"
                to="mailto:mikelikesrobots@outlook.com"
              >
                <i className="fa fa-envelope fa-lg padding-right--sm"></i>
                mikelikesrobots@outlook.com
              </Link>
            </div>
          </div>
        </section>

        {/* Capabilities */}
        <section className={clsx("container padding-vert--md")}>
          <hr></hr>
          <div className={clsx(styles.capabilityCardRow)}>
            {CapabilityCardDefs.map((x) => (
              <CapabilityCard {...x} />
            ))}
          </div>
        </section>

        {/* Customer Testimonials */}
        <section className={clsx("container padding-vert--md")}>
          <hr></hr>
          <h2>Customer Testimonials</h2>
          {TestimonialDefs.map((x) => (
            <CustomerTestimonial {...x} />
          ))}
        </section>

        {/* Previous Projects */}
        <section className={clsx("container padding-vert--md")}>
          <hr></hr>
          <h2>Previous Projects</h2>
          {PastProjectDefs.map((x) => (
            <PastProject {...x} />
          ))}
        </section>
      </main>
    </Layout>
  );
}
