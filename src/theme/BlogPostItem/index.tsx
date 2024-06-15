import React from 'react'
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import { useBlogPost, useColorMode } from '@docusaurus/theme-common/internal'
import BlogPostItem from '@theme-original/BlogPostItem'
import Giscus from "@giscus/react";

export default function BlogPostItemWrapper(props) {
  const { siteConfig } = useDocusaurusContext();
  const { metadata } = useBlogPost()
  const { colorMode } = useColorMode()
  const { frontMatter } = metadata
  const { comments = true, slug, title } = frontMatter

  const { pathname } = useLocation();
  const containsSlug = pathname.indexOf(slug) > -1;

  return (
    <>
      <BlogPostItem {...props} />
      {comments && containsSlug && (
        <>
          <meta name="giscus:backlink" content={`${siteConfig.url}${pathname}`}></meta>
          <Giscus
            repo="mikelikesrobots/mikelikesrobots.github.io"
            repoId="R_kgDOK5fkWQ"
            category="Comments"
            categoryId="DIC_kwDOK5fkWc4CgGP9"
            mapping="pathname"
            term={title}
            strict="0"
            reactionsEnabled="1"
            emitMetadata="1"
            inputPosition="top"
            theme={colorMode}
            lang="en"
            loading="lazy"
          />
        </>
      )}
    </>
  )
}
