
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import FeaturedBlogs from '@site/src/components/FeaturedBlogs';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import SiteHighlights from '@site/src/components/SiteHighlights';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx(styles.flexbox)}>
        <div>
          <img className={styles.mascot} src="/img/ike_u_fullsize.png" />
        </div>
        <div className={clsx(styles.flexgrow)}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  return (
    <Layout
      title={`Home`}
      description="Tutorials and resources for connecting robots to the cloud">
      
      <HomepageHeader />
      <main>
        <FeaturedBlogs />
        <div className="padding--md" />
        <SiteHighlights />
        <div className="padding--md" />
        </main>
    </Layout>
  );
}
