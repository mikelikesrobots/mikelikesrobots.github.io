import clsx from "clsx";
import styles from "./index.module.css";

type PastProjectProps = {
  title: string;
  client: string;
  objective: string;
  details: string[];
  outcome: string;
  image?: string;
  alt?: string;
};

function PastProject(props: PastProjectProps) {
  const maybeImage = props.image ? (
    <div className={clsx(styles.projectImage)}>
      <img alt={props.alt} src={props.image} />
    </div>
  ) : null;
  return (
    <div className={clsx(styles.projectBox)}>
      <details>
        <summary>{props.title}</summary>
        <div className={clsx(styles.projectDetails)}>
          <div className={clsx(styles.projectTextDetails)}>
            <p>
              <strong>Client:</strong> {props.client}
            </p>
            <p>
              <strong>Objective:</strong> {props.objective}
            </p>

            <h4>Project Details</h4>
            <ul>
              {props.details.map((x) => (
                <li>{x}</li>
              ))}
            </ul>

            <p>
              <strong>Outcome:</strong> {props.outcome}
            </p>
          </div>
          {maybeImage}
        </div>
      </details>
    </div>
  );
}

export { PastProjectProps, PastProject };
