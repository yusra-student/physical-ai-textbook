import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import ModuleBoxes from '../components/ModuleBoxes'; // Import the new component

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1-ros2/lesson1">
            Start Your Journey
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Build the Future with ${siteConfig.title}`}
      description="An interactive guide to Physical AI and Humanoid Robotics. Learn ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action systems.">
      <HomepageHeader />
      <main>
        <ModuleBoxes /> {/* Render the new component */}
      </main>
    </Layout>
  );
}
