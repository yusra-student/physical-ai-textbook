// API service for communicating with the backend
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const uploadBook = async (file, bookId, title, author) => {
  const formData = new FormData();
  formData.append('file', file);
  formData.append('book_id', bookId);
  formData.append('title', title);
  if (author) formData.append('author', author);

  const response = await fetch(`${API_BASE_URL}/ingest`, {
    method: 'POST',
    body: formData,
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.message || error.error || 'Upload failed');
  }

  return await response.json();
};

export const searchQuery = async (query) => {
  const response = await fetch(`${API_BASE_URL}/query`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ query }),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.message || error.error || 'Search failed');
  }

  return await response.json();
};

export const askQuestion = async (query, documentId) => {
  const response = await fetch(`${API_BASE_URL}/ask`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      query,
      document_id: documentId || 1  // Default document ID if not provided
    }),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.message || error.error || 'Ask failed');
  }

  return await response.json();
};

export const getIngestionStatus = async (jobId) => {
  const response = await fetch(`${API_BASE_URL}/ingest/status/${jobId}`);

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.message || error.error || 'Status check failed');
  }

  return await response.json();
};