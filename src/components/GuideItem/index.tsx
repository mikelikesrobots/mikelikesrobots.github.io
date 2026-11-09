import styles from './styles.module.css';
import YouTubeVideo from '@site/src/components/YouTubeVideo';
import Image from '@theme/ThemedImage';
import Link from '@docusaurus/Link';

export type GuideItemProps = {
  title: string;
  headingLevel?: 4 | 5;
  summary?: string;
  youtube?: string;
  local_blog?: string;
  aws_community?: string;
  github?: string;
}

type ResourceKind = 'guide' | 'aws' | 'github';

const resourceTypes: Array<{kind: ResourceKind; label: string; source: string; icon: string}> = [
  {kind: 'guide', label: 'Written guide', source: 'Mike Likes Robots', icon: '/img/logo.svg'},
  {kind: 'aws', label: 'AWS Community article', source: 'community.aws', icon: '/img/aws-community-icon-180x180.png'},
  {kind: 'github', label: 'Source code', source: 'GitHub repository', icon: '/img/social/github.svg'},
];

function Summary(props: Pick<GuideItemProps, 'summary'>): JSX.Element | null {
  return props.summary ?
    <p>{props.summary}</p> :
    null;
}

function YoutubeVideo(props: Pick<GuideItemProps, 'youtube' | 'title'>): JSX.Element {
  const id = props.youtube?.match(/\/embed\/([^?]+)/)?.[1];
  return id ? <YouTubeVideo videoId={id} title={props.title} showCaption={false} /> : null;
}

function ResourceIcon({kind, icon}: {kind: ResourceKind; icon: string}): JSX.Element {
  return <Image alt="" className={`${styles.resourceIcon} ${styles[`resourceIcon--${kind}`]}`} sources={{light: icon, dark: icon}} />;
}

export function ResourceKey(): JSX.Element {
  return <aside className={styles.resourceKey} aria-label="Resource icon key">
    <strong>Resource links</strong>
    <ul>{resourceTypes.map(resource => <li key={resource.kind}>
      <ResourceIcon kind={resource.kind} icon={resource.icon} />
      <span><strong>{resource.label}</strong><small>{resource.source}</small></span>
    </li>)}</ul>
  </aside>;
}

function ResourceLink({href, title, kind, label, source, icon}: {
  href?: string;
  title: string;
  kind: ResourceKind;
  label: string;
  source: string;
  icon: string;
}): JSX.Element | null {
  return href ? <li>
    <Link className={styles.resourceLink} to={href} title={`${label}: ${title}`} aria-label={`${label} for ${title} on ${source}`}>
      <ResourceIcon kind={kind} icon={icon} />
      <span className="sr-only">{label}: {title}</span>
    </Link>
  </li> : null;
}

function TextLinks(props: Pick<GuideItemProps, 'title' | 'local_blog' | 'aws_community' | 'github'>): JSX.Element {
  if (!props.local_blog && !props.aws_community && !props.github) return null;
  return <ul className={styles.resourceLinks} aria-label={`Resources for ${props.title}`}>
    {resourceTypes.map(resource => <ResourceLink key={resource.kind} href={resource.kind === 'guide' ? props.local_blog : resource.kind === 'aws' ? props.aws_community : props.github} title={props.title} {...resource} />)}
  </ul>;
}

export default function GuideItem(props: GuideItemProps): JSX.Element {
  const Title = `h${props.headingLevel || 5}` as 'h4' | 'h5';
  return (
    <section className={styles.card}>
      <div className={styles.card_title}>
        <YoutubeVideo {...props} />
        <Title className={styles.h5}>{props.title}</Title>
      </div>
      <div className={styles.card_container}>
        <Summary {...props} />
        <TextLinks {...props} />
      </div>
    </section>
  )
}
