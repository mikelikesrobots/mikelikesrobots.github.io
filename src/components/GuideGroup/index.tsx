import styles from './styles.module.css';

export type GuideGroupProps = {
    children: React.ReactNode;
}

export default function GuideGroup(props: GuideGroupProps): JSX.Element {
    return <div className={styles.wrapper}>{props.children}</div>
}