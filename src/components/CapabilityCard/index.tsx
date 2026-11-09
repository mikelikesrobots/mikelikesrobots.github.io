import clsx from "clsx";
import styles from "./index.module.css";

type CapabilityCardProps = {
  title: string;
  description: string | JSX.Element;
};

function CapabilityCard({ title, description }: CapabilityCardProps) {
  return (
    <div className={clsx(styles.capabilityCardWrapper)}>
      <div
        className={clsx("padding--md", styles.capabilityCard)}
      >
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export {CapabilityCardProps, CapabilityCard};
