import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

type BlogDetails = {
  title: string;
  title_href?: string;
  description: JSX.Element;
};

const FeatureList: BlogDetails[] = [
    {
        title: "Why would I connect my robots to the cloud?",
        title_href: '/blog/why-use-cloud-robotics',
        description: <>Why connect my robots to the cloud at all? What benefits does it bring me, and what...</>,
    },
    {
        title: "Controlling Robots using a Large Language Model",
        title_href: '/blog/llm-robot-control',
        description: <>See how to use LLMs to control robots, with more flexible commands and feedback...</>,
    },
    {
        title: "Controlling a 6DOF Robot Arm with ros2_control",
        title_href: '/blog/6dof-arm-ros2-control',
        description: <>Understand the ros2_control example code for a 6DOF robot arm, and how to adapt it...</>,
    },
];

function FeaturedBlog({title, title_href, description}: BlogDetails) {
  const titleElement = title_href ?
    <Heading as="h2"><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h2">{title}</Heading>;
  return (
    <div className={clsx(styles.featureCardWrapper)}>
      <div className={clsx("padding--md item shadow--md", styles.featureCard)}>
        {titleElement}
        <p className={styles.featureCardGrow}>{description}</p>
        <div className={clsx("text--center")}>
          <Link
              className="button button--secondary button--sm item shadow--md text--center"
              to={title_href}>
              Read More
            </Link>
          </div>
      </div>
    </div>
  );
}

export default function FeaturedBlogs(): JSX.Element {
  return (
    <section className={clsx("container padding-vert--md", styles.featuredBlogs)}>
      <h1 className={clsx("text--center padding-vert--md")}>Featured Blogs</h1>
      <div className={clsx("padding-horiz--lg", styles.featureCardRow)}>
          {FeatureList.map((props, idx) => (
            <FeaturedBlog key={idx} {...props} />
          ))}
      </div>
      <div className={clsx("text--center padding-vert--lg")}>
          <Link
            className="button button--secondary button--lg item shadow--md"
            to="/blog">
            More Blog Posts
          </Link>
        </div>
    </section>
  );
}
