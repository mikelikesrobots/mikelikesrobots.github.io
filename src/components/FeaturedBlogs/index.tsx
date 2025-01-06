import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

type FeatureItem = {
  title: string;
  title_href?: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
    {
        title: "Why would I connect my robots to the cloud?",
        title_href: '/blog/why-use-cloud-robotics',
        description: <p>Why connect my robots to the cloud at all? What benefits does it bring me, and what trade-offs...</p>,
    },
    {
        title: "Simulating Robots in the Cloud with EC2 and O3DE",
        title_href: '/blog/simulation-in-cloud',
        description: <p>With simulations, we can repeatedly run a robot through exactly the same setup every...</p>,
    },
    {
        title: "Building a ros2_control System",
        title_href: '/blog/jetbot-motors-pt2',
        description: <p>The next step in making ros2_control work with the WaveShare JetBot...</p>,
    },
];

function FeaturedBlog({title, title_href, description}: FeatureItem) {
  const titleElement = title_href ?
    <Heading as="h2"><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h2">{title}</Heading>;
  return (
    <div className={clsx(styles.featureCardWrapper)}>
      <div className={clsx("padding--md item shadow--md", styles.featureCard)}>
        {titleElement}
        <p className={styles.featureCardGrow}>{description}</p>
        <div className={clsx("text--center ")}>
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

export default function HomepageFeatures(): JSX.Element {
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
