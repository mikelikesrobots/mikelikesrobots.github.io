import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useColorMode} from '@docusaurus/theme-common';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import BlogPostItem from '@theme-original/BlogPostItem';
import BlogPostItemHeaderInfo from '@theme/BlogPostItem/Header/Info';
import ReadMoreLink from '@theme/BlogPostItem/Footer/ReadMoreLink';
import TagsListInline from '@theme/TagsListInline';
import PreviewTitle from './Header/Title';
import PreviewContent from './Content';
import Giscus from '@giscus/react';
import styles from './styles.module.css';

export default function BlogPostItemWrapper(props: React.ComponentProps<typeof BlogPostItem>) {
  const {siteConfig} = useDocusaurusContext();
  const {metadata, isBlogPostPage} = useBlogPost();
  const {colorMode} = useColorMode();
  const {comments = true} = metadata.frontMatter;
  return <>
    {isBlogPostPage ? <BlogPostItem {...props} /> : <article className={[styles.previewItem, props.className].filter(Boolean).join(' ')}>
      <header>
        <PreviewTitle />
        <BlogPostItemHeaderInfo />
      </header>
      <PreviewContent>{props.children}</PreviewContent>
      {(metadata.tags.length > 0 || metadata.hasTruncateMarker) &&
        <footer className={styles.previewMeta}>
          {metadata.tags.length > 0 && <div className={styles.previewTags}>
            <TagsListInline tags={metadata.tags} />
          </div>}
          {metadata.hasTruncateMarker &&
            <ReadMoreLink className={styles.readMore} blogPostTitle={metadata.title} to={metadata.permalink} />}
        </footer>}
    </article>}
    {comments && isBlogPostPage && <>
      <meta name="giscus:backlink" content={`${siteConfig.url}${metadata.permalink}`} />
      <Giscus repo="mikelikesrobots/mikelikesrobots.github.io" repoId="R_kgDOK5fkWQ"
        category="Comments" categoryId="DIC_kwDOK5fkWc4CgGP9" mapping="pathname"
        strict="0" reactionsEnabled="1" emitMetadata="1" inputPosition="top"
        theme={colorMode} lang="en" loading="lazy" />
    </>}
  </>;
}
