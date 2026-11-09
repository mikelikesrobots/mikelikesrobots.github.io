import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

type Highlight = {
  title: string;
  title_href?: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  Icon?: JSX.Element;
  description: JSX.Element;
};

const HighlightList: Highlight[] = [
  {
    title: 'Blog',
    title_href: '/blog',
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
    title_href: 'https://www.youtube.com/@mikelikesrobots',
    Icon: <span className={styles.featureSvg} aria-hidden="true">▶</span>,
    description: (
      <>
        All videos are hosted on YouTube, including tutorials and labs, coding videos,
        build videos, and career advice.
      </>
    ),
  },
  {
    title: 'GitHub',
    title_href: 'https://github.com/mikelikesrobots',
    Icon: <span className={styles.featureSvg} aria-hidden="true">&lt;/&gt;</span>,
    description: (
      <>
        Any video or blog post with more than a few lines of code will have the full source code
        put up on GitHub, so you can look through the code and try it for yourself.
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

function SiteHighlight({title, title_href, Svg, Icon, description}: Highlight) {
  const titleElement = title_href ?
    <Heading as="h3" className={styles.pad_top}><Link to={title_href}>{title}</Link></Heading> :
    <Heading as="h3" className={styles.pad_top}>{title}</Heading>;
  const Img = Svg ? <Svg className={styles.featureSvg} aria-hidden="true" focusable="false" /> : Icon;
  return (
    <div className={clsx('col col--3')}>
      <Link to={title_href} className={styles.hover_expand} aria-label={title}>
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

export default function SiteHighlights(): JSX.Element {
  return (
    <section className={clsx("padding-vert--lg", styles.homepageFeaturesWrapper)}>
      <div className="container">
        <Heading as="h2" className="text--center padding-vert--sm">More ways to learn</Heading>
        <div className={clsx("row", styles.homepageFeatures)}>
          {HighlightList.map((props, idx) => (
            <SiteHighlight key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
