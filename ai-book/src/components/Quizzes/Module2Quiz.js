import React, { useState } from 'react';
import styles from './quiz.module.css'; // Import the new CSS module
import clsx from 'clsx';


const quizQuestions = [
  {
    question: "What is a primary benefit of using robotic simulation?",
    options: ["Increased hardware costs", "Faster deployment to real robots", "Safe and repeatable testing environments", "Requires less computational power"],
    answer: 2, // Index of "Safe and repeatable testing environments"
  },
  {
    question: "Which component of Gazebo is responsible for simulating realistic rigid body dynamics?",
    options: ["3D Graphics", "Sensor Simulation", "Physics Engine", "Plugin Architecture"],
    answer: 2, // Index of "Physics Engine"
  },
  {
    question: "What file format is commonly used to describe a robot's physical structure for simulation in Gazebo?",
    options: ["YAML", "JSON", "URDF/Xacro", "SDF"],
    answer: 2, // Index of "URDF/Xacro"
  },
  {
    question: "Which Gazebo feature allows for extending its functionality with custom robot control or sensor models?",
    options: ["Physics Engine", "3D Graphics", "ROS Integration", "Plugin Architecture"],
    answer: 3, // Index of "Plugin Architecture"
  },
  {
    question: "Which of these is NOT a common sensor type simulated in robotics?",
    options: ["LiDAR", "Camera", "IMU", "Emotional Sensor"],
    answer: 3, // Index of "Emotional Sensor"
  },
  {
    question: "What is the purpose of the `libgazebo_ros_camera.so` plugin?",
    options: ["To control camera movement", "To render camera images in Gazebo", "To publish simulated camera data to ROS 2 topics", "To adjust camera focus"],
    answer: 2, // Index of "To publish simulated camera data to ROS 2 topics"
  },
  {
    question: "What is the name of the bridge package that facilitates communication between ROS 2 and Gazebo?",
    options: ["ros_bridge", "gazebo_ros_pkgs", "ros_gz", "ros2_gazebo_connector"],
    answer: 2, // Index of "ros_gz"
  },
  {
    question: "Which ROS 2 message type is commonly used to send velocity commands to a differential drive robot?",
    options: ["sensor_msgs/msg/LaserScan", "geometry_msgs/msg/Twist", "nav_msgs/msg/Odometry", "std_msgs/msg/String"],
    answer: 1, // Index of "geometry_msgs/msg/Twist"
  },
  {
    question: "What tool is typically used to visualize ROS 2 sensor data (e.g., camera images, LiDAR scans)?",
    options: ["Gazebo", "rviz2", "VS Code", "rqt_graph"],
    answer: 1, // Index of "rviz2"
  },
  {
    question: "When integrating a sensor into a robot's URDF/Xacro model, where do Gazebo-specific sensor definitions reside?",
    options: ["Inside the `<link>` tag", "Inside the `<joint>` tag", "Within a `<gazebo reference='link_name'>` block", "In a separate `sensor.yaml` file"],
    answer: 2, // Index of "Within a `<gazebo reference='link_name'>` block"
  },
];

export default function Module2Quiz() {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [showScore, setShowScore] = useState(false);
  const [score, setScore] = useState(0);
  const [selectedAnswer, setSelectedAnswer] = useState(null);
  const [quizEnded, setQuizEnded] = useState(false);

  const handleAnswerOptionClick = (optionIndex) => {
    setSelectedAnswer(optionIndex);
    const isCorrect = optionIndex === quizQuestions[currentQuestion].answer;

    if (isCorrect) {
      setScore(score + 1);
    }

    setTimeout(() => {
      const nextQuestion = currentQuestion + 1;
      if (nextQuestion < quizQuestions.length) {
        setCurrentQuestion(nextQuestion);
        setSelectedAnswer(null); // Reset selected answer for next question
      } else {
        setShowScore(true);
        setQuizEnded(true);
      }
    }, 1000); // Short delay to show feedback
  };

  const resetQuiz = () => {
    setCurrentQuestion(0);
    setShowScore(false);
    setScore(0);
    setSelectedAnswer(null);
    setQuizEnded(false);
  };

  return (
    <div className={styles.quizContainer}>
      {showScore ? (
        <div className={styles.scoreSection}>
          You scored {score} out of {quizQuestions.length}
          <button onClick={resetQuiz} className={clsx('button button--primary', styles.resetButton)}>
            Retake Quiz
          </button>
        </div>
      ) : (
        <>
          <div className={styles.questionSection}>
            <div className={styles.questionCount}>
              <span>Question {currentQuestion + 1}</span>/{quizQuestions.length}
            </div>
            <div className={styles.questionText}>{quizQuestions[currentQuestion].question}</div>
          </div>
          <div className={styles.answerSection}>
            {quizQuestions[currentQuestion].options.map((option, index) => (
              <button
                key={index}
                onClick={() => handleAnswerOptionClick(index)}
                className={clsx(
                  selectedAnswer === index && (
                    index === quizQuestions[currentQuestion].answer ? styles.correct : styles.incorrect
                  ),
                  selectedAnswer !== null && index !== quizQuestions[currentQuestion].answer && styles.fadedIncorrect // Dim incorrect options if an answer is selected
                )}
                disabled={selectedAnswer !== null} // Disable buttons after an answer is selected
              >
                {option}
              </button>
            ))}
          </div>
        </>
      )}
    </div>
  );
}
