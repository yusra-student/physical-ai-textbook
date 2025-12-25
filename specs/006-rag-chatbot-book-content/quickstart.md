# Quick Start Guide: RAG Chatbot for Published Books

## Overview
This guide provides a quick start for developers to set up and run the RAG Chatbot for Published Books application locally. This system allows users to upload books, ask questions about the content, and receive AI-generated answers with citations.

## Prerequisites
- Python 3.11+
- Node.js 16+ (for frontend)
- Docker and Docker Compose
- Cohere API key
- Qdrant Cloud account (or local instance)
- Neon PostgreSQL account (or local instance)

## Environment Setup

### 1. Clone and Navigate
```bash
git clone <repository-url>
cd hackatone-ai-book
cd ai-book  # Navigate to the frontend directory
```

### 2. Install Backend Dependencies
```bash
cd backend  # Navigate to the backend directory
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Install Frontend Dependencies
```bash
cd frontend  # Navigate to the frontend directory
npm install
```

### 4. Set Up Environment Variables
Create a `.env` file in the backend directory with the following variables:

```env
# Backend Environment Variables
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
DATABASE_URL=postgresql://username:password@localhost:5432/rag_chatbot
NEON_DATABASE_URL=your_neon_database_url
SECRET_KEY=your_secret_key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

## Running the Application

### 1. Start External Services with Docker
```bash
# In the project root
docker-compose -f docker/docker-compose.yml up -d
```

### 2. Run the Backend
```bash
cd backend
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Run the Frontend
```bash
cd frontend
npm run dev
```

The backend will be available at `http://localhost:8000` and the frontend at `http://localhost:3000`.

## API Endpoints

### 1. Book Ingestion
Upload and process a book:
```bash
curl -X POST "http://localhost:8000/ingest" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/book.pdf" \
  -F "book_id=book123" \
  -F "title=Sample Book Title" \
  -F "author=Sample Author"
```

Check ingestion status:
```bash
curl -X GET "http://localhost:8000/ingest/status/{job_id}"
```

### 2. Query Processing (Full Book)
Ask questions about the entire book:
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main theme of the book?",
    "document_id": "doc_uuid_789",
    "session_id": "session_uuid_999",
    "top_k": 5,
    "confidence_threshold": 0.7
  }'
```

### 3. Query Processing (User Selection)
Ask questions about user-selected text:
```bash
curl -X POST "http://localhost:8000/query-selection" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this passage mean?",
    "selection_id": "selection_uuid_222",
    "selected_text": "It was a beautiful morning and the sun was rising...",
    "session_id": "session_uuid_999"
  }'
```

### 4. Conversation Management
Start a new conversation:
```bash
curl -X POST "http://localhost:8000/conversations" \
  -H "Content-Type: application/json" \
  -d '{
    "document_id": "doc_uuid_789",
    "user_id": "user_123"
  }'
```

Get conversation history:
```bash
curl -X GET "http://localhost:8000/conversations/{session_id}"
```

## Frontend Integration

### 1. Embedding the Chatbot Widget
The React component can be easily embedded in any webpage:

```jsx
import ChatbotWidget from './components/ChatbotWidget';

function App() {
  return (
    <div className="App">
      <h1>My Book Reader</h1>
      <BookContent />
      <ChatbotWidget bookId="book123" />
    </div>
  );
}
```

### 2. Text Selection Feature
The chatbot automatically detects text selections in the book reader and provides focused querying:

```javascript
// The frontend automatically captures text selections
document.addEventListener('mouseup', () => {
  const selectedText = window.getSelection().toString();
  if (selectedText.length > 10) {
    // "Ask about selection" button becomes active
  }
});
```

## Data Models Overview

### Document/Book
- Represents an ingested book with metadata
- Fields: `id`, `book_id`, `title`, `author`, `ingestion_status`, etc.

### Text Chunk
- Segments of book content processed for RAG
- Fields: `id`, `document_id`, `text`, `chapter_num`, `page_start`, `page_end`, etc.

### Conversation
- Tracks user interactions with the system
- Fields: `id`, `session_id`, `document_id`, `query`, `answer`, `confidence_score`, etc.

## Testing the Application

### 1. Run Backend Unit Tests
```bash
cd backend
pytest tests/unit/ -v
```

### 2. Run Integration Tests
```bash
cd backend
pytest tests/integration/ -v
```

### 3. Run Frontend Tests
```bash
cd frontend
npm test
```

## Configuration

### Performance Tuning
- Adjust `top_k` parameter to balance between accuracy and performance (default: 5)
- Modify `confidence_threshold` to control answer quality (default: 0.7)
- Configure batch size for embedding generation (default: 100)

### Rate Limiting
- Default: 1000 requests/hour per user
- 10,000 queries/day per book
- 100 requests/minute per IP

## Common Issues and Troubleshooting

### 1. Database Connection Issues
- Ensure Neon PostgreSQL connection string is correctly configured
- Check that the database is accessible from your environment

### 2. Vector Search Issues
- Verify Qdrant API key and URL are correctly set
- Check that embeddings have been properly generated for your book

### 3. Cohere API Issues
- Ensure your Cohere API key has sufficient quota
- Check that the embedding and LLM models are available in your region

### 4. File Upload Issues
- Verify that the file is a valid PDF or EPUB (under 500MB)
- Check that the backend has sufficient storage for temporary files

## Next Steps

1. **Production Deployment**: Use the provided Docker configuration for deployment
2. **Monitoring**: Set up logging and metrics as specified in the implementation plan
3. **Security**: Implement proper authentication and authorization for production use
4. **Scaling**: Monitor usage and upgrade cloud tiers as needed

For more detailed documentation, refer to the `docs/` directory in the repository.