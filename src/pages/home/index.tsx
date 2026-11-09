
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import FeaturedBlogs from '@site/src/components/FeaturedBlogs';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';

import styles from './index.module.css';
import SiteHighlights from '@site/src/components/SiteHighlights';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={clsx('container', styles.flexbox)}>
        <div>
          <img className={styles.mascot} src="/img/ike_u_fullsize.png" alt="" width="128" height="128" />
        </div>
        <div className={styles.introduction}>
          <Heading as="h1" className="hero__title">
            Practical robotics, explained.
          </Heading>
          <p className="hero__subtitle">Tutorials on ROS 2, robot control, and cloud robotics.</p>
          <div className={styles.actions}>
            <Link className="button button--primary button--lg" to="/docs/getting-started/intro">Start learning</Link>
            <Link className={styles.workLink} to="/hireme">Work with me →</Link>
          </div>
          <p className={styles.motto}>{siteConfig.tagline}</p>
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
      
      <main>
        <HomepageHeader />
        <FeaturedBlogs />
        <SiteHighlights />
      </main>
    </Layout>
  );
}
