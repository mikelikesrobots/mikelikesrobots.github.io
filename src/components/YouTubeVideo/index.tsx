import React, {useRef, useState} from 'react';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import styles from './styles.module.css';

export function videoThumbnail(videoId: string) {
  return `https://i.ytimg.com/vi/${videoId}/hqdefault.jpg`;
}

export default function YouTubeVideo({videoId, title, start = 0, showCaption = true}: {
  videoId: string; title: string; start?: number; showCaption?: boolean;
}) {
  const [playing, setPlaying] = useState(false);
  const frame = useRef<HTMLIFrameElement>(null);
  return <><div className={styles.video}>
    {playing ? <iframe ref={frame}
      src={`https://www.youtube-nocookie.com/embed/${videoId}?autoplay=1&start=${start}`}
      title={title} allow="autoplay; encrypted-media; picture-in-picture; fullscreen"
      allowFullScreen onLoad={() => frame.current?.focus()} /> :
      <button type="button" className={styles.poster} onClick={() => setPlaying(true)}
        aria-label={`Play video: ${title}`}>
        <img src={videoThumbnail(videoId)} alt="" width="480" height="360" loading="lazy" />
        <span className={styles.play} aria-hidden="true" />
        {showCaption && <span className={styles.caption}>Watch: {title}</span>}
      </button>}
  </div>
    <noscript><p><a href={`https://www.youtube.com/watch?v=${videoId}&t=${start}`}>Watch {title} on YouTube</a></p></noscript>
  </>;
}

export function BlogVideo() {
  const {metadata, isBlogPostPage} = useBlogPost();
  const videoId = metadata.frontMatter.youtube as string | undefined;
  return isBlogPostPage && videoId ? <YouTubeVideo videoId={videoId} title={metadata.title} /> : null;
}
