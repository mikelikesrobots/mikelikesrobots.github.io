import styles from './styles.module.css';
import Details from '@theme/Details';
import Image from '@theme/ThemedImage';
import Link from '@docusaurus/Link';

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
    <Details summary="Watch on YouTube!">
      <iframe className="youtube-video" src={props.youtube} title="YouTube video player" frameBorder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen></iframe>
    </Details> :
    null;
}

function LocalBlogLink(props: {href?: string}): JSX.Element {
  return props.href ? 
    <div className={styles.text_link}>
      <Link to={props.href}>
        <Image height={"30px"} className={styles.pad_right} sources={{light: "/img/logo.svg", dark: "/img/logo.svg"}}/>
        Read more on Mike Likes Robots Blog!
      </Link>
    </div> :
    null;
}

function AWSCommunityBlogLink(props: {href?: string}): JSX.Element {
  return props.href ? 
    <div className={styles.text_link}>
      <Link to={props.href}>
        <Image height={"30px"} className={styles.pad_right} sources={{light: "/img/aws_logo_smile_1200x630.png", dark: "/img/aws_logo_smile_1200x630.png"}}/>
        Read more on AWS Community!
      </Link>
    </div> :
    null;
}

function GithubLink(props: Pick<GuideItemProps, 'github'>): JSX.Element {
  return props.github ? 
    <div className={styles.text_link}>
      <Link to={props.github}>
        <Image height={"30px"} className={styles.pad_right} sources={{light: "/img/github.jpg", dark: "/img/github.jpg"}}/>
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
        <h5 className={styles.h5}>{props.title}</h5>
      </div>
      <div className={styles.card_container}>
        <Summary {...props} />
        <TextLinks {...props} />
        <YoutubeVideo {...props} />
      </div>
    </section>
  )
}