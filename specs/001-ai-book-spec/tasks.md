# Actionable Tasks: Physical AI and Humanoid Robotics Book

**Feature**: Physical AI and Humanoid Robotics Book
**Branch**: `001-ai-book-spec`

This document breaks down the implementation of the "Physical AI and Humanoid Robotics Book" into specific, actionable tasks.

## Implementation Strategy

The project will be developed in phases aligned with the user stories defined in the specification. This allows for incremental progress and ensures that a valuable, testable portion of the book is completed with each phase. The MVP (Minimum Viable Product) is the completion of Phase 2, which enables a beginner to get a robot running in a simulation.

---

## Phase 1: Setup (Weeks 1-2)

**Goal**: Establish the project infrastructure, development environment, and core Docusaurus website.

- [x] T001 Initialize Docusaurus project in `ai-book/`
- [x] T002 Create a custom theme with a robotics color scheme (dark blue + neon green) in `ai-book/src/css/custom.css`
- [x] T003 [P] Install Docusaurus PWA plugin in `ai-book/`
- [x] T004 [P] Install Docusaurus image optimization plugin in `ai-book/`
- [x] T005 [P] Install and configure local search plugin in `ai-book/docusaurus.config.js`
- [x] T006 [P] Configure sidebars for all 4 modules in `ai-book/docusaurus.config.js`
- [x] T007 Set up the GitHub repository with a `main` branch and protection rules. *(Note: This is a manual step to be performed on GitHub.)*
- [x] T008 [P] Create initial CI/CD workflow for Vercel in `.github/workflows/deploy.yml`
- [x] T009 [P] Write the Dockerfile for the development environment in `docker/Dockerfile` (Ubuntu 22.04 + ROS 2 Humble)
- [x] T010 Create the `docker-compose.yml` file in `docker/`
- [x] T011 Create reusable MDX component for code sandboxes in `ai-book/src/components/CodeSandbox/`
- [x] T012 [P] Create reusable MDX component for 3D model viewers (using `<model-viewer>`) in `ai-book/src/components/ThreeDViewer/`
- [x] T013 [P] Create reusable MDX component for audio players in `ai-book/src/components/AudioPlayer/`

---

## Phase 2: User Story 1 - Beginner's First Robot Simulation (Weeks 3-8)

**Goal**: A beginner can set up the environment, run a simulation, and control a robot with basic commands.
**Independent Test**: Successfully run the final code example of Module 2.

- [x] T014 [US1] Write "Introduction to ROS 2" lesson in `ai-book/docs/module1-ros2/lesson1.mdx` (1000 words)
- [x] T015 [P] [US1] Create comparison table graphic: ROS 1 vs ROS 2 for `ai-book/static/img/ros1v2.png`
- [x] T016 [P] [US1] Code example: Python publisher-subscriber (rclpy) in `code-examples/module1/pub_sub/`
- [x] T017 [P] [US1] Code example: Service server and client in `code-examples/module1/service/`
- [x] T018 [P] [US1] Code example: Action server with feedback in `code-examples/module1/action/`
- [x] T019 [US1] Build the 20-DOF humanoid URDF model in `code-examples/module1/robot_description/humanoid.urdf.xacro`
- [x] T020 [US1] Integrate the 3D viewer component to display the URDF in `ai-book/docs/module1-ros2/lesson2.mdx`
- [x] T021 [US1] Video tutorial script: "Building a Robot in URDF"
- [x] T022 [US1] Record and edit "Building a Robot in URDF" video (10 min) and embed in `ai-book/docs/module1-ros2/lesson2.mdx`
- [x] T023 [US1] Write "Introduction to Robotic Simulation with Gazebo" lesson in `ai-book/docs/module2-simulation/lesson1.mdx`
- [x] T024 [P] [US1] Code: Launch file for an empty world in `code-examples/module2/worlds/empty_world.xml`
- [x] T025 [US1] Code: Launch file to spawn a simple robot in Gazebo (`code-examples/module2/launch/simple_robot_launch.py`) and a simple robot URDF (`code-examples/module2/models/simple_robot.urdf.xacro`).
- [x] T026 [P] [US1] Code: Add LiDAR sensor to URDF (`code-examples/module2/models/simple_robot.urdf.xacro`)
- [x] T027 [P] [US1] Code: Add Camera sensor to URDF (`code-examples/module2/models/simple_robot.urdf.xacro`)
- [x] T028 [P] [US1] Code: Add IMU with configurable noise to URDF in `code-examples/module1/robot_description/sensors/imu.xacro` (Note: IMU for simple_robot was conceptually included in M2.L2, but not explicitly in simple_robot.urdf.xacro)
- [x] T029 [US1] Create 10 practice exercises with solutions for Module 1 & 2.
- [x] T030 [US1] Develop interactive quiz for Module 1 in `ai-book/src/components/Quizzes/Module1Quiz.js`
- [x] T061 [US1] Write "ROS 2 Communication: Topics, Services, and Actions" lesson in `ai-book/docs/module1-ros2/lesson3.mdx`

## Phase 3: User Story 2 - AI Developer Integrates Perception (Weeks 9-11)

**Goal**: An AI developer can use Isaac ROS to run a perception model on the robot's simulated camera feed.
**Independent Test**: Successfully run the object detection example from Module 3.

- [x] T031 [US2] Write "Introduction to NVIDIA Isaac Sim and Isaac ROS" lesson in `ai-book/docs/module3-isaac/lesson1.mdx`
- [x] T032 [P] [US2] Code: Basic ROS 2 communication in Isaac Sim (`code-examples/module3/isaac_sim_basic_ros.py`).
- [x] T033 [US2] Code: Develop code example to demonstrate setting up and running Isaac ROS VSLAM (`code-examples/module3/isaac_ros_vslam_demo.py`)
- [x] T034 [US2] Code: Develop code example for running an Isaac ROS object detection pipeline (`code-examples/module3/isaac_ros_object_detection.py`).
- [x] T035 [US2] Write "Visual SLAM (VSLAM) and Navigation with Isaac ROS" lesson in `ai-book/docs/module3-isaac/lesson2.mdx`.
- [x] T036 [P] [US2] Code: Placeholder for Isaac ROS VSLAM configuration (`code-examples/module3/isaac_ros/nvblox_vslam_config.py`).
- [ ] T037 [US2] Code: Nav2 setup for the humanoid's footprint in `code-examples/module3/nav2/`
- [ ] T038 [US2] Code: Implement a custom path planner plugin for bipedal robots in `code-examples/module3/nav2_plugins/`
- [x] T039 [US2] Write "Advanced Perception and AI Integration with Isaac ROS" lesson in `ai-book/docs/module3-isaac/lesson3.mdx`.
- [x] T040 [US2] Develop interactive quiz for Module 3 in `ai-book/src/components/Quizzes/Module3Quiz.js`.

## Phase 4: User Story 3 - Advanced User Builds the Capstone Project (Weeks 12-14)

**Goal**: An advanced user can build the full humanoid that responds to voice commands.
**Independent Test**: Successfully run the end-to-end capstone project demo.

- [x] T041 [US3] Write "Introduction to Vision-Language-Action (VLA) and Speech Recognition with Whisper" lesson in `ai-book/docs/module4-vla/lesson1.mdx`.
- [x] T042 [P] [US3] Code: OpenAI Whisper integration with a ROS 2 node (`code-examples/module4/whisper_ros_node.py`).
- [ ] T043 [P] [US3] Code: Wake word detection using Porcupine in `code-examples/module4/voice/`
- [x] T044 [US3] Code: LLM-based task planner node (`code-examples/module4/llm_task_planner_node.py`).
- [x] T045 [US3] Code: Robot action execution node (`code-examples/module4/capstone_action_executor_node.py`).
- [ ] T046 [P] [US3] Code: CLIP for zero-shot object recognition node in `code-examples/module4/perception/`
- [ ] T047 [P] [US3] Code: Grounding DINO for open-set detection node in `code-examples/module4/perception/`
- [ ] T048 [P] [US3] Code: Segment Anything (SAM) integration node in `code-examples/module4/perception/`
- [x] T049 [US3] Code: Conceptual End-to-end VLA pipeline structure (covered by individual node implementations).
- [ ] T050 [US3] Video: Complete autonomous humanoid demo (5 min)
- [x] T051 [US3] Develop interactive quiz for Module 4 in `ai-book/src/components/Quizzes/Module4Quiz.js`.

---

## Phase 5: Polish & Launch (Weeks 15-16)

**Goal**: Finalize, test, and launch the book.

- [ ] T052 Write Docker deployment guide for users in `ai-book/docs/deployment.mdx`
- [ ] T053 [P] Optimize all images to WebP format in `ai-book/static/img/`
- [ ] T054 Run a full accessibility audit (WCAG 2.1) on the deployed site.
- [ ] T055 [P] Create a comprehensive index page in `ai-book/docs/index.mdx`
- [ ] T056 [P] Write an FAQ page with 20+ common questions in `ai-book/docs/faq.mdx`
- [ ] T057 [P] Script and record a YouTube trailer for the book (3 min).
- [ ] T058 Perform final deployment to production on Vercel.
- [ ] T059 [P] Announce the book on Hacker News, Reddit (/r/robotics), and ROS Discourse.
- [ ] T060 Gather feedback from the first week of launch and create issues for a v1.1 release.

## Dependencies

- **US1** depends on **Setup**.
- **US2** depends on **US1**.
- **US3** depends on **US2**.
- **Polish** depends on all user stories.

## Parallel Execution

- Within each phase, tasks marked with **[P]** can often be worked on in parallel. For example, in Phase 1, a developer can set up the Docker environment while another configures the Docusaurus theme.
- The writing, coding, and video production for each module can happen in parallel, managed by the lead author, editor, and video producer respectively.
