import styles from './styles.module.css';
import Image from '@theme/ThemedImage';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

export type GuideItemProps = {
  title: string;
  summary?: string;
  youtube?: string;
  local_blog?: string;
  aws_community?: string;
  github?: string;
}

function Summary(props: Pick<GuideItemProps, 'summary'>): JSX.Element {
  return props.summary ?
    <p>{props.summary}</p> :
    null;
}

function YoutubeVideo(props: Pick<GuideItemProps, 'youtube'>): JSX.Element {
  return props.youtube ?
    <iframe className={clsx("youtube-video", styles.rounded_top_corners)} src={props.youtube} title="YouTube video player" frameBorder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen></iframe> :
    null;
}

function LocalBlogLink(props: {href?: string}): JSX.Element {
  return props.href ? 
    <div className={styles.text_link}>
      <Link to={props.href}>
        <Image alt="Ike-U - mascot icon for blog" height={"30px"} className={styles.pad_right} sources={{light: "/img/logo.svg", dark: "/img/logo.svg"}}/>
        Read more on my blog!
      </Link>
    </div> :
    null;
}

function AWSCommunityBlogLink(props: {href?: string}): JSX.Element {
  return props.href ? 
    <div className={styles.text_link}>
      <Link to={props.href}>
        <Image alt="AWS Community Logo Icon" height={"30px"} className={styles.pad_right} sources={{light: "/img/aws_logo_smile_bullet.webp", dark: "/img/aws_logo_smile_bullet.webp"}}/>
        Read more on AWS Community!
      </Link>
    </div> :
    null;
}

function GithubLink(props: Pick<GuideItemProps, 'github'>): JSX.Element {
  return props.github ? 
    <div className={styles.text_link}>
      <Link to={props.github}>
        <Image alt="Github Logo Icon" height={"30px"} className={styles.pad_right} sources={{light: "/img/github_bullet.webp", dark: "/img/github_bullet.webp"}}/>
        See the source code on Github!
      </Link>
    </div> :
    null;
}

function TextLinks(props: Pick<GuideItemProps, 'local_blog' | 'aws_community' | 'github'>): JSX.Element {
  const local = <LocalBlogLink href={props.local_blog} />;
  const aws = <AWSCommunityBlogLink href={props.aws_community} />;
  const github = <GithubLink {...props} />
  if (local !== null || aws !== null || github !== null) {
    return <div className={styles.text_links_wrapper}>
      {local}
      {aws}
      {github}
    </div>
  } else {
    return null;
  }
}

export default function GuideItem(props: GuideItemProps): JSX.Element {
  return (
    <section className={styles.card}>
      <div className={styles.card_title}>
        <YoutubeVideo {...props} />
        <p className={styles.h5}>{props.title}</p>
      </div>
      <div className={styles.card_container}>
        <Summary {...props} />
        <TextLinks {...props} />
      </div>
    </section>
  )
}