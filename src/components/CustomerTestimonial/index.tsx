import clsx from "clsx";
import styles from "./index.module.css";

type CustomerTestimonialProps = {
  testimonial: string;
  customerName: string;
  position: string;
  company: string;
};

function CustomerTestimonial(props: CustomerTestimonialProps) {
  return (
    <div className={clsx(styles.testimonialBox)}>
      <p className={clsx(styles.testimonialText)}>{props.testimonial}</p>
      <p className={clsx(styles.testimonialAuthor)}>
        — <strong>{props.customerName}</strong>, {props.position},{" "}
        {props.company}
      </p>
    </div>
  );
}

export { CustomerTestimonialProps, CustomerTestimonial };
