import React from 'react';
import './App.css';
import BookSearch from './components/BookSearch';
import BookUpload from './components/BookUpload';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>AI Book Assistant</h1>
        <p>Search and interact with your AI knowledge base</p>
      </header>
      <main>
        <BookUpload />
        <BookSearch />
      </main>
    </div>
  );
}

export default App;