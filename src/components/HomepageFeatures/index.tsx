import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

type FeatureItem = {
  title: string;
  title_href?: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  Icon?: JSX.Element;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Blog',
    title_href: '/docs/getting-started/intro',
    Svg: require('@site/static/img/blog.svg').default,
    description: (
      <>
        The blog is actively updated with technical guides and tutorials. Keep
        checking back for new posts, and to see how they fit in with Getting Started.
      </>
    ),
  },
  {
    title: 'YouTube Channel',
    title_href: '/blog',
    Icon: <i className={clsx("fab fa-youtube fa-lg", styles.featureSvg)}></i>,
    description: (
      <>
        All videos are hosted on YouTube, including tutorials and labs, coding videos,
        build videos, and career advice.
      </>
    ),
  },
  {
    title: 'Github',
    title_href: '/blog',
    Icon: <i className={clsx("fab fa-github fa-lg", styles.featureSvg)}></i>,
    description: (
      <>
        Any video or blog post with more than a few lines of code will have the full source code
        put up on Github, so you can look through the code and try it for yourself.
      </>
    ),
  },
  {
    title: 'Getting Started Guide',
    title_href: '/docs/getting-started/intro',
    Svg: require('@site/static/img/shuttle.svg').default,
    description: (
      <>
        Getting Started has videos and blog posts to help you connect your robot to the cloud and get value from your data.
      </>
    ),
  },
];

function Feature({title, title_href, Svg, Icon, description}: FeatureItem) {
  const titleElement = title_href ?
    <Heading as="h2" className={styles.pad_top}><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h2" className={styles.pad_top}>{title}</Heading>;
  const Img = Svg ? <Svg className={styles.featureSvg} role="img" /> : Icon;
  return (
    <div className={clsx('col col--3')}>
      <Link to={title_href} className={styles.hover_expand}>
        <div className={clsx("text--center", styles.svg_background)}>
          {Img}
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
    <section className={clsx("container", "padding-vert--lg", styles.homepageFeaturesWrapper)}>
      <Heading as="h1" className="text--center padding-vert--sm">Site Features</Heading>
      <div className={clsx("row", styles.homepageFeatures)}>
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))}
      </div>
    </section>
  );
}
