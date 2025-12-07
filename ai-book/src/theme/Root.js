import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget'; // Adjust path if necessary

// Default implementation, that you can customize
// Injected by Docusaurus: `props.children`
function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}

export default Root;
