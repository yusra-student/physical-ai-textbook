import React, { useState } from 'react';
import styles from './quiz.module.css'; // Import the new CSS module
import clsx from 'clsx';


const quizQuestions = [
  {
    question: "What is the primary benefit of NVIDIA Isaac Sim for robotics development?",
    options: ["Low computational requirements", "Easy physical robot deployment", "Physically accurate, scalable simulation", "Exclusive support for C++ robotics"],
    answer: 2, // Index of "Physically accurate, scalable simulation"
  },
  {
    question: "Which technology does Isaac Sim leverage for photorealistic graphics and synthetic data generation?",
    options: ["OpenGL", "Vulkan", "NVIDIA RTX", "DirectX"],
    answer: 2, // Index of "NVIDIA RTX"
  },
  {
    question: "What is Isaac ROS?",
    options: ["A programming language for robots", "A collection of hardware-accelerated packages for ROS 2", "A robotic operating system for space exploration", "A simulation environment for drones"],
    answer: 1, // Index of "A collection of hardware-accelerated packages for ROS 2"
  },
  {
    question: "VSLAM is primarily concerned with:",
    options: ["Robot arm manipulation", "Simultaneous localization and mapping using visual data", "Voice command recognition", "Object grasping in unstructured environments"],
    answer: 1, // Index of "Simultaneous localization and mapping using visual data"
  },
  {
    question: "Which Isaac ROS package would typically be used for high-performance object detection?",
    options: ["isaac_ros_vslam", "isaac_ros_nav2", "isaac_ros_detectnet", "isaac_ros_image_proc"],
    answer: 2, // Index of "isaac_ros_detectnet"
  },
  {
    question: "Why is synthetic data generation from Isaac Sim valuable for AI development?",
    options: ["It's always more realistic than real data", "It reduces the need for physical sensors", "It provides large, diverse datasets with ground truth annotations", "It eliminates the need for deep learning models"],
    answer: 2, // Index of "It provides large, diverse datasets with ground truth annotations"
  },
  {
    question: "What is the main role of Nav2 in an autonomous robot system that uses VSLAM?",
    options: ["Performing visual feature extraction", "Managing the robot's hardware interfaces", "Providing high-level navigation planning and control", "Simulating sensor data"],
    answer: 2, // Index of "Providing high-level navigation planning and control"
  },
  {
    question: "Which of these is an example of an advanced perception task accelerated by Isaac ROS?",
    options: ["Basic motor control", "Text-to-speech conversion", "Semantic segmentation", "Simple data logging"],
    answer: 2, // Index of "Semantic segmentation"
  },
  {
    question: "Integrating AI outputs like object detections into robotic decision-making helps robots to:",
    options: ["Only avoid obstacles", "Understand and act upon their environment in a meaningful way", "Reduce their power consumption", "Increase their communication range"],
    answer: 1, // Index of "Understand and act upon their environment in a meaningful way"
  },
  {
    question: "What does the 'GPU-accelerated' aspect of Isaac ROS imply?",
    options: ["It requires a special robotic arm", "It runs faster on CPUs", "It utilizes NVIDIA GPUs for enhanced performance", "It can only be used with physical robots"],
    answer: 2, // Index of "It utilizes NVIDIA GPUs for enhanced performance"
  },
];

export default function Module3Quiz() {
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
