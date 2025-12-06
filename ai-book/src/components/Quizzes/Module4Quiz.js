import React, { useState } from 'react';
import styles from './quiz.module.css'; // Import the new CSS module
import clsx from 'clsx';


const quizQuestions = [
  {
    question: "What does VLA stand for in the context of robotics?",
    options: ["Visual Learning Algorithm", "Volatile Logic Array", "Vision-Language-Action", "Virtual Lab Automation"],
    answer: 2, // Index of "Vision-Language-Action"
  },
  {
    question: "Which of these is a key advantage of the VLA paradigm?",
    options: ["Reduced need for complex sensors", "More intuitive human-robot collaboration", "Elimination of programming in robotics", "Faster robot movement in industrial settings"],
    answer: 1, // Index of "More intuitive human-robot collaboration"
  },
  {
    question: "What is OpenAI's Whisper model primarily used for in robotics?",
    options: ["Generating robot speech", "Image recognition", "Speech recognition (audio to text)", "Robot navigation"],
    answer: 2, // Index of "Speech recognition (audio to text)"
  },
  {
    question: "Why are LLMs considered beneficial for high-level task planning in robotics?",
    options: ["They are faster than traditional algorithms", "They can understand and decompose complex natural language goals", "They provide precise low-level motor control", "They reduce the need for sensor data"],
    answer: 1, // Index of "They can understand and decompose complex natural language goals"
  },
  {
    question: "What is 'prompt engineering' when using LLMs for robotic tasks?",
    options: ["Designing the robot's physical structure", "Crafting input queries to guide the LLM's action generation", "Developing robot programming languages", "Debugging LLM code"],
    answer: 1, // Index of "Crafting input queries to guide the LLM's action generation"
  },
  {
    question: "Which sensor modality primarily provides 3D structural information for a robot's perception?",
    options: ["RGB Cameras", "Microphones", "LIDAR / Depth Sensors", "Accelerometers"],
    answer: 2, // Index of "LIDAR / Depth Sensors"
  },
  {
    question: "In a VLA system, what typically happens after a human voice command is transcribed by Whisper?",
    options: ["The robot immediately executes a hardcoded action", "The text is fed into an LLM-based task planner", "The robot asks for clarification from the human", "The robot updates its internal map"],
    answer: 1, // Index of "The text is fed into an LLM-based task planner"
  },
  {
    question: "What role does the 'Action Execution Node' play in a capstone VLA project?",
    options: ["Transcribing speech", "Generating high-level action plans", "Translating LLM plans into low-level robot movements", "Visualizing sensor data"],
    answer: 2, // Index of "Translating LLM plans into low-level robot movements"
  },
  {
    question: "Which of these is NOT a challenge that LLMs help address in traditional robotic planning?",
    options: ["Ambiguity in natural language", "Handling novel situations", "Lack of common sense reasoning", "Ensuring real-time sensor data fusion"],
    answer: 3, // Index of "Ensuring real-time sensor data fusion"
  },
  {
    question: "By combining vision, language, and action, robots can achieve:",
    options: ["Only faster object manipulation", "A more limited understanding of their environment", "More intuitive and flexible human-robot collaboration", "Reduced power consumption across all tasks"],
    answer: 2, // Index of "More intuitive and flexible human-robot collaboration"
  },
];

export default function Module4Quiz() {
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
