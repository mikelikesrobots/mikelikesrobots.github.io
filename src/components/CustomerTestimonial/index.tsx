import clsx from "clsx";
import styles from "./index.module.css";

type CustomerTestimonialProps = {
  testimonial: string[];
  customerName: string;
  position: string;
  company: string;
};

function CustomerTestimonial(props: CustomerTestimonialProps) {
  return (
    <div className={clsx(styles.testimonialBox)}>
      <blockquote className={styles.quote}>
        <p className={styles.testimonialText}>{props.testimonial[0]}</p>
        {props.testimonial.length > 1 && <details>
          <summary>Read full testimonial<span className="sr-only"> from {props.customerName}</span></summary>
          {props.testimonial.slice(1).map(paragraph => <p key={paragraph} className={styles.testimonialText}>{paragraph}</p>)}
        </details>}
      </blockquote>
      <p className={clsx(styles.testimonialAuthor)}>
        — <strong>{props.customerName}</strong>, {props.position},{" "}
        {props.company}
      </p>
    </div>
  );
}

export { CustomerTestimonialProps, CustomerTestimonial };
