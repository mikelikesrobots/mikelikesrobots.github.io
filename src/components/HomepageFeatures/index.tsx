import clsx from 'clsx';
import Heading from '@theme/Heading';
import ThemedImage from '@theme/ThemedImage';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

type FeatureItem = {
  title: string;
  title_href?: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Github',
    title_href: '/docs/getting-started/intro',
    Svg: require('@site/static/img/shuttle.svg').default,
    description: (
      // <>
      //   Getting Started shows how to get going with connecting your robots to
      //   the cloud. It has resources including videos and blogs on AWS cloud
      //   connections, setting up robots, and getting value from your data.
      // </>
      <p>Getting Started Guide</p>
    ),
  },
  {
    title: 'Active Blogging Platform',
    title_href: '/blog',
    Svg: require('@site/static/img/blog.svg').default,
    description: (
      // <>
      //   Site includes a blog actively updated as new content is released. Keep
      //   checking back for more articles, and to see how they fit in with Getting
      //   Started!
      // </>
      <p>Cool Blog</p>
    ),
  },
];

function Feature({title, title_href, Svg, description}: FeatureItem) {
  const titleElement = title_href ?
    <Heading as="h2" className={styles.pad_top}><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h2" className={styles.pad_top}>{title}</Heading>;
  return (
    <div className={clsx('col col--3')}>
      <Link to={title_href} className={styles.hover_expand}>
        <div className={clsx("text--center", styles.svg_background)}>
          <Svg className={styles.featureSvg} role="img" />
        </div>
      </Link>
      <div className="text--center padding-horiz--md"> 
        {titleElement}
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={clsx("container", "padding-vert--md", styles.homepageFeatures)}>
      {/* WHat do I want to put here? Some images? */}
      {/* Small circles with logos that expand on hover over */}
      {/* We've got blog, guide, Github, and YouTube */}
      {/* So it's similar to what was there before */}
      <div className="row">
        <p>Hello, I am here to talk about all the cool things on this website.</p>
        {/* {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))} */}
      </div>
      <div className="row">
        <div className='col col--3'>
          <p>The blog is super cool.</p>
        </div>
        <div className='col col--3'>
          <p>The YouTube channel is super cool.</p>
        </div>
        <div className='col col--3'>
          <p>The Github is super cool.</p>
        </div>
        <div className='col col--3'>
          <p>The Getting Started Guide is super cool.</p>
        </div>
      </div>
    </section>
  );
}
