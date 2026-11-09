import React from 'react';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItemFooter from '@theme-original/BlogPostItem/Footer';
import TagsListInline from '@theme/TagsListInline';

export default function BlogPostItemFooterWrapper() {
  const {metadata, isBlogPostPage} = useBlogPost();
  if (isBlogPostPage) return <BlogPostItemFooter />;
  return metadata.tags.length ? <footer className="docusaurus-mt-lg">
    <TagsListInline tags={metadata.tags} />
  </footer> : null;
}
