import React from 'react';
import Link from '@docusaurus/Link';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItemContent from '@theme-original/BlogPostItem/Content';
import BlogThumbnail from '@site/src/components/BlogThumbnail';
import styles from './styles.module.css';

export default function BlogPostItemContentWrapper(props: React.ComponentProps<typeof BlogPostItemContent>) {
  const {metadata, isBlogPostPage} = useBlogPost();
  const {youtube, thumbnail} = metadata.frontMatter;
  const videoId = typeof youtube === 'string' ? youtube : undefined;
  const fallback = typeof thumbnail === 'string' ? thumbnail : undefined;
  const content = <BlogPostItemContent {...props} />;

  if (isBlogPostPage || (!videoId && !fallback)) return content;

  return <div className={styles.previewRow}>
    <Link className={styles.thumbnailLink} to={metadata.permalink} aria-label={`Read: ${metadata.title}`}>
      <BlogThumbnail className={styles.thumbnail} youtube={videoId} thumbnail={fallback} source={metadata.source} width={480} height={360} />
    </Link>
    <div className={styles.postText}>{content}</div>
  </div>;
}
