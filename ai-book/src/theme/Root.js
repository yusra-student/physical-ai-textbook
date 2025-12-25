import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget'; // Adjust path if necessary
import { AuthProvider } from '../components/Auth/AuthProvider';

// Default implementation, that you can customize
// Injected by Docusaurus: `props.children`
function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatbotWidget />
    </AuthProvider>
  );
}

export default Root;
