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
      <img alt={props.alt} src={props.image} loading="lazy" width="400" height="300" />
    </div>
  ) : null;
  return (
    <div className={clsx(styles.projectBox)}>
      <h3>{props.title}</h3>
      <p><strong>Outcome:</strong> {props.outcome}</p>
      <details>
        <summary>Project details<span className="sr-only">: {props.title}</span></summary>
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
                <li key={x}>{x}</li>
              ))}
            </ul>


          </div>
          {maybeImage}
        </div>
      </details>
    </div>
  );
}

export { PastProjectProps, PastProject };
