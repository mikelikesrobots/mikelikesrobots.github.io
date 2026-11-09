import React from 'react';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItemHeaderAuthors from '@theme-original/BlogPostItem/Header/Authors';

export default function BlogPostItemHeaderAuthorsWrapper(props: React.ComponentProps<typeof BlogPostItemHeaderAuthors>) {
  const {isBlogPostPage} = useBlogPost();
  return isBlogPostPage ? <BlogPostItemHeaderAuthors {...props} /> : null;
}
