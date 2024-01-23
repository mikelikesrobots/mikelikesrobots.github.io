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
    title: 'Thorough Getting Started Guide',
    title_href: '/docs/getting-started/intro',
    Svg: require('@site/static/img/shuttle.svg').default,
    description: (
      <>
        Getting Started shows how to get going with connecting your robots to
        the cloud. It has resources including videos and blogs on AWS cloud
        connections, setting up robots, and getting value from your data.
      </>
    ),
  },
  {
    title: 'Active Blogging Platform',
    title_href: '/blog',
    Svg: require('@site/static/img/blog.svg').default,
    description: (
      <>
        Site includes a blog actively updated as new content is released. Keep
        checking back for more articles, and to see how they fit in with Getting
        Started!
      </>
    ),
  },
];

function Feature({title, title_href, Svg, description}: FeatureItem) {
  const titleElement = title_href ?
    <Heading as="h3" className={styles.pad_top}><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h3" className={styles.pad_top}>{title}</Heading>;
  return (
    <div className={clsx('col col--6')}>
      <Link to={title_href} className={styles.hover_expand}>
        <div className={clsx("text--center", styles.svg_background)}>
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center padding-horiz--md">
          {titleElement}
          <p>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
