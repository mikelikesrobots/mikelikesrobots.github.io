import React from 'react';
import Link from '@docusaurus/Link';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItemContent from '@theme-original/BlogPostItem/Content';
import {videoThumbnail} from '@site/src/components/YouTubeVideo';
import styles from './styles.module.css';

export default function BlogPostItemContentWrapper(props: React.ComponentProps<typeof BlogPostItemContent>) {
  const {metadata, isBlogPostPage} = useBlogPost();
  const {youtube} = metadata.frontMatter;
  const content = <BlogPostItemContent {...props} />;

  if (isBlogPostPage || typeof youtube !== 'string') return content;

  return <div className={styles.previewRow}>
    <Link className={styles.thumbnailLink} to={metadata.permalink} aria-label={`Read: ${metadata.title}`}>
      <img className={styles.thumbnail} src={videoThumbnail(youtube)} alt="" width="480" height="360" loading="lazy" />
    </Link>
    <div className={styles.postText}>{content}</div>
  </div>;
}
