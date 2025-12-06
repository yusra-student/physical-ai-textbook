import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';
import React from 'react';

type ModuleItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: JSX.Element;
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Dive into the fundamentals of the Robot Operating System 2 (ROS 2), learning about its architecture,
        communication mechanisms like topics, services, and actions, and how to define robot models.
      </>
    ),
    link: '/docs/module1-ros2/lesson1',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Explore robotic simulation with Gazebo and Unity. Learn to set up virtual environments, integrate sensors,
        and establish seamless communication between simulations and ROS 2.
      </>
    ),
    link: '/docs/module2-simulation/lesson1',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Unleash the power of NVIDIA Isaac Sim and Isaac ROS for accelerating AI in robotics. Dive into VSLAM
        for localization and mapping, and advanced perception with GPU-accelerated models.
      </>
    ),
    link: '/docs/module3-isaac/lesson1',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Re-using SVG for now
    description: (
      <>
        Culminate your learning with the Vision-Language-Action (VLA) paradigm. Integrate speech recognition (Whisper),
        LLM-based task planning, and multimodal perception for intelligent humanoid robotics.
      </>
    ),
    link: '/docs/module4-vla/lesson1',
  },
];

function Module({title, Svg, description, link}: ModuleItem) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <div className="text--center">
        <Svg className={styles.moduleSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <Link
          className={clsx(
            'button button--primary button--lg',
            styles.moduleButton,
          )}
          to={link}>
          Start Learning
        </Link>
      </div>
    </div>
  );
}

export default function ModuleBoxes(): JSX.Element {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}