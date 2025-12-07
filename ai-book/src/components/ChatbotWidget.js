import React, { useState, useEffect, useRef } from 'react';
import './ChatbotWidget.css'; // Import the CSS file

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async () => {
    if (inputMessage.trim() === '') return;

    const userMessage = { text: inputMessage, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:5000/chat', { // Flask backend URL
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: inputMessage }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage = { text: data.response, sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error("Error sending message to backend:", error);
      setMessages((prevMessages) => [
        ...prevMessages,
        { text: "Sorry, I'm having trouble connecting right now.", sender: 'bot' },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleSendMessage();
    }
  };

  return (
    <>
      <button className="chatbot-button" onClick={toggleChat}>
        {isOpen ? 'X' : 'Chat'}
      </button>

      {isOpen && (
        <div className="chatbot-modal">
          <div className="chatbot-header">
            <h3>AI Book Chat</h3>
            <button className="chatbot-close-button" onClick={toggleChat}>X</button>
          </div>
          <div className="chatbot-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`chatbot-message chatbot-message-${msg.sender}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && <div className="chatbot-message chatbot-message-bot">Typing...</div>}
            <div ref={messagesEndRef} />
          </div>
          <div className="chatbot-input-area">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask me anything about the book..."
              disabled={isLoading}
            />
            <button onClick={handleSendMessage} disabled={isLoading}>Send</button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
