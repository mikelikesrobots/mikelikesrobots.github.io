import React, {useState} from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {videoThumbnail} from '@site/src/components/YouTubeVideo';
import thumbnails from '@generated/article-metadata/default/thumbnails';

export type ThumbnailMetadata = {youtube?: string; thumbnail?: string; source?: string};

export default function BlogThumbnail({youtube, thumbnail, source: postSource, ...props}: ThumbnailMetadata & {
  className?: string; width: number; height: number;
}) {
  const [failedSource, setFailedSource] = useState<string>();
  const fallback = useBaseUrl(thumbnails[postSource] || thumbnail || '');
  const videoSource = youtube ? videoThumbnail(youtube) : undefined;
  const source = videoSource && failedSource !== videoSource
    ? videoSource
    : thumbnail ? fallback : undefined;

  if (!source) return null;

  return <img {...props} src={source} alt="" loading="lazy"
    onError={source === videoSource && thumbnail ? () => setFailedSource(source) : undefined} />;
}
