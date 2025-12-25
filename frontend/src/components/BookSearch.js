import React, { useState } from 'react';
import { askQuestion } from '../services/api';

const BookSearch = () => {
  const [query, setQuery] = useState('');
  const [searching, setSearching] = useState(false);
  const [results, setResults] = useState(null);
  const [error, setError] = useState('');

  const handleSearch = async (e) => {
    e.preventDefault();
    if (!query.trim()) {
      setError('Please enter a search query');
      return;
    }

    setSearching(true);
    setError('');
    setResults(null);

    try {
      const data = await askQuestion(query, 1); // Using document ID 1 as default
      setResults(data);
    } catch (err) {
      setError(`Error: ${err.message}`);
    } finally {
      setSearching(false);
    }
  };

  return (
    <div>
      <h2>Search Your AI Knowledge Base</h2>
      <form onSubmit={handleSearch} className="query-form">
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask anything about AI..."
          className="query-input"
        />
        <button type="submit" disabled={searching}>
          {searching ? 'Searching...' : 'Ask AI'}
        </button>
      </form>

      {error && (
        <div className="results">
          <h3>Error</h3>
          <p>{error}</p>
        </div>
      )}

      {results && (
        <div className="results">
          <h3>Results for: "{query}"</h3>
          <div>
            <h4>Answer:</h4>
            <p>{results.response}</p>

            <h4>Context:</h4>
            <ul>
              {results.context.map((context, index) => (
                <li key={index}>
                  <p><strong>Score: {context.score}</strong></p>
                  <p>{context.text.substring(0, 300)}{context.text.length > 300 ? '...' : ''}</p>
                </li>
              ))}
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

export default BookSearch;