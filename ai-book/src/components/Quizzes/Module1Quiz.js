import React, { useState } from 'react';
import styles from './quiz.module.css'; // Import the new CSS module
import clsx from 'clsx';

const quizQuestions = [
  {
    question: "What is the primary communication mechanism in ROS 2 for continuous data streams?",
    options: ["Services", "Actions", "Topics", "Parameters"],
    answer: 2, // Index of "Topics"
  },
  {
    question: "Which file format is commonly used to describe a robot's physical structure in ROS?",
    options: ["SDF", "YAML", "URDF", "XML"],
    answer: 2, // Index of "URDF"
  },
  {
    question: "What is the main advantage of using synthetic data from simulators like Isaac Sim?",
    options: ["Lower computational cost", "Easier to collect and annotate", "Higher realism", "Direct deployment to hardware"],
    answer: 1, // Index of "Easier to collect and annotate"
  },
  {
    question: "Which ROS 2 concept is used for long-running, goal-oriented tasks with periodic feedback?",
    options: ["Topics", "Services", "Actions", "Parameters"],
    answer: 2, // Index of "Actions"
  },
  {
    question: "What does DDS stand for in the context of ROS 2?",
    options: ["Data Distribution Service", "Distributed Data System", "Direct Data Stream", "Dynamic Data Sync"],
    answer: 0, // Index of "Data Distribution Service"
  },
  {
    question: "What is a 'Node' in ROS 2?",
    options: ["A physical robot", "A process that performs computation", "A communication channel", "A type of sensor"],
    answer: 1, // Index of "A process that performs computation"
  },
  {
    question: "Which command would you use to view active ROS 2 topics?",
    options: ["ros2 node list", "ros2 topic list", "ros2 service list", "ros2 param list"],
    answer: 1, // Index of "ros2 topic list"
  },
  {
    question: "What is the purpose of an 'xacro' file in ROS?",
    options: ["To define ROS 2 parameters", "To compress URDF files", "To enable macro-based XML generation for URDF", "To configure ROS 2 launches"],
    answer: 2, // Index of "To enable macro-based XML generation for URDF"
  },
  {
    question: "What is RViz used for in ROS 2?",
    options: ["Robot programming", "3D visualization of sensor data and robot state", "Package management", "Inter-process communication"],
    answer: 1, // Index of "3D visualization of sensor data and robot state"
  },
  {
    question: "Which communication pattern is best for requesting a single piece of information and waiting for a response?",
    options: ["Topics", "Services", "Actions", "Parameters"],
    answer: 1, // Index of "Services"
  },
];

export default function Module1Quiz() {
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
