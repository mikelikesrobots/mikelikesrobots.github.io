import React from 'react';
import clsx from 'clsx';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItemHeaderTitle from '@theme-original/BlogPostItem/Header/Title';
import styles from './styles.module.css';

export default function BlogPostItemHeaderTitleWrapper(props: React.ComponentProps<typeof BlogPostItemHeaderTitle>) {
  const {isBlogPostPage} = useBlogPost();
  return <BlogPostItemHeaderTitle {...props} className={clsx(props.className, !isBlogPostPage && styles.previewTitle)} />;
}
