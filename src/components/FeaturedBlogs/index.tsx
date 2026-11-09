import React from 'react';
import Link from '@docusaurus/Link';
import {usePluginData} from '@docusaurus/useGlobalData';
import {videoThumbnail} from '@site/src/components/YouTubeVideo';
import styles from './styles.module.css';

type ArticleMetadata = {title: string; url: string; description: string; date: string; youtube?: string};
type FeaturedArticle = Pick<ArticleMetadata, 'title' | 'url' | 'description' | 'youtube'> & {topic: string};

const featuredArticles: FeaturedArticle[] = [
  {
    title: 'Autonomously Mapping with Visual SLAM',
    url: '/blog/autonomous-viam-rover',
    description: 'Build an autonomous indoor mapping rover with ROS 2, RealSense depth sensing, Visual SLAM, and Nav2.',
    topic: 'ROS 2 project',
  },
  {
    title: 'Cost-Effective Robotics Simulation in the Cloud',
    url: '/blog/ec2-spot-sims',
    description: 'Build an automated, cost-conscious AWS environment for running O3DE, Gazebo, and NVIDIA Isaac Sim on GPU Spot Instances.',
    topic: 'Cloud robotics',
  },
  {
    title: 'Understanding PID Controllers',
    url: '/blog/understand-pid-controllers',
    description: 'Develop an intuitive understanding of PID control and experiment with an interactive controller visualisation.',
    topic: 'Robot control',
  },
];

function ArticleImage({article, className}: {article: FeaturedArticle | ArticleMetadata; className?: string}) {
  return article.youtube
    ? <img className={className || styles.thumbnail} src={videoThumbnail(article.youtube)} alt="" width="480" height="270" loading="lazy" />
    : <div className={`${className || styles.topicImage} ${styles.topicImage}`} aria-hidden="true">{'topic' in article ? article.topic : 'Robotics'}</div>;
}

export default function FeaturedBlogs() {
  const {posts} = usePluginData('article-metadata') as {posts: ArticleMetadata[]};
  const latest = posts.find(post => post.url.startsWith('/blog/'));

  return <>
    {latest && <section className={`padding-vert--lg ${styles.latestSection}`} aria-labelledby="latest-article-heading">
      <div className="container">
        <p className={styles.sectionEyebrow}>Latest article</p>
        <article className={styles.latestArticle}>
          <Link className={styles.latestImageLink} to={latest.url} tabIndex={-1} aria-hidden="true">
            <ArticleImage article={latest} className={styles.latestImage} />
          </Link>
          <div className={styles.latestBody}>
            <h2 id="latest-article-heading"><Link to={latest.url}>{latest.title}</Link></h2>
            {latest.description && <p>{latest.description}</p>}
            <Link className={styles.readLink} to={latest.url} aria-label={`Read the latest article: ${latest.title}`}>Read the latest article →</Link>
          </div>
        </article>
      </div>
    </section>}

    <section className={`padding-vert--lg ${styles.featuredBlogs}`} aria-labelledby="featured-tutorials-heading">
      <div className="container">
        <h2 id="featured-tutorials-heading">Featured tutorials</h2>
        <p>Explore practical guides covering ROS 2, cloud robotics, and robot control.</p>
        <nav className="topic-links" aria-label="Explore tutorial topics">
          <Link to="/blog/tags/ros-2">ROS 2</Link><Link to="/blog/tags/cloud">Cloud Robotics</Link><Link to="/blog/tags/control">Robot Control</Link>
        </nav>
        <div className={styles.featureCardRow}>{featuredArticles.map(article => {
          const metadata = posts.find(post => post.url === article.url);
          const card = {...article, youtube: metadata?.youtube};
          return <article className={styles.featureCard} key={article.url}>
            <Link to={article.url} tabIndex={-1} aria-hidden="true"><ArticleImage article={card} /></Link>
            <div className={styles.cardBody}>
              <span className={styles.cardTopic}>{article.topic}</span>
              <h3><Link to={article.url}>{article.title}</Link></h3>
              <p>{article.description}</p>
              <Link className={styles.readLink} to={article.url} aria-label={`Read: ${article.title}`}>Read article →</Link>
            </div>
          </article>;
        })}</div>
        <div className="text--center margin-top--lg"><Link className="button button--primary" to="/blog">Browse all articles</Link></div>
      </div>
    </section>
  </>;
}
