# Data Model: RAG Chatbot for Published Books

## Overview
This document defines the data model for the RAG Chatbot for Published Books feature, based on the entities and requirements identified in the feature specification.

## Entity Definitions

### 1. Document/Book
**Description**: Represents an ingested book with metadata and processing information

**Fields**:
- `id` (UUID): Primary identifier for the document
- `book_id` (String): Unique identifier for the book (externally provided)
- `title` (String, 500): Title of the book
- `author` (String, 500): Author of the book
- `publisher` (String, 500): Publisher of the book (optional)
- `published_date` (Date): Original publication date (optional)
- `total_pages` (Integer): Number of pages in the book (optional)
- `file_url` (Text): URL or path to the original book file
- `ingestion_status` (Enum): Status of ingestion process ('pending', 'processing', 'completed', 'failed')
- `created_at` (DateTime): Timestamp when document was created
- `updated_at` (DateTime): Timestamp when document was last updated
- `metadata` (JSONB): Additional metadata about the book in JSON format

**Relationships**:
- One Document has many Chunks (one-to-many)
- One Document has many Conversations (one-to-many)
- One Document has many User Selections (one-to-many)

**Validation Rules**:
- `book_id` must be unique
- `title` is required and cannot exceed 500 characters
- `ingestion_status` must be one of the defined enum values
- `created_at` defaults to current timestamp

### 2. Text Chunk
**Description**: Represents a segment of a book content that has been processed and vectorized

**Fields**:
- `id` (UUID): Primary identifier for the chunk
- `document_id` (UUID): Foreign key to the Document table
- `chunk_index` (Integer): Sequential index of this chunk within the document
- `text` (Text): The actual text content of the chunk
- `chapter_num` (Integer): Chapter number where this chunk appears (optional)
- `section_name` (String, 500): Name of the section where this chunk appears (optional)
- `page_start` (Integer): Starting page number of this chunk (optional)
- `page_end` (Integer): Ending page number of this chunk (optional)
- `token_count` (Integer): Number of tokens in this chunk
- `vector_id` (String): Unique identifier for the vector in Qdrant (optional)
- `created_at` (DateTime): Timestamp when chunk was created

**Relationships**:
- Many Chunks belong to one Document (many-to-one)
- Many Conversations reference multiple Chunks through source_chunk_ids (many-to-many via JSONB)

**Validation Rules**:
- `document_id` must reference a valid Document
- `text` is required and represents the content to be searched
- `chunk_index` should be sequential within a document
- `page_start` and `page_end` should be consistent if provided

### 3. User Selection
**Description**: Represents a specific text selection made by a user for focused querying

**Fields**:
- `id` (UUID): Primary identifier for the user selection
- `document_id` (UUID): Foreign key to the Document table
- `session_id` (String): Session identifier associated with this selection (optional)
- `selected_text` (Text): The actual text that was selected by the user
- `start_position` (Integer): Start position of the selection in the document
- `end_position` (Integer): End position of the selection in the document
- `qdrant_namespace` (String): Namespace in Qdrant for this specific selection's vectors
- `created_at` (DateTime): Timestamp when selection was created

**Relationships**:
- Many User Selections belong to one Document (many-to-one)

**Validation Rules**:
- `document_id` must reference a valid Document
- `selected_text` is required
- `start_position` and `end_position` define the selection range
- `qdrant_namespace` must be unique

### 4. Conversation
**Description**: Represents a user's interaction session with the system, containing query-answer pairs

**Fields**:
- `id` (UUID): Primary identifier for the conversation
- `session_id` (String): Unique identifier for the conversation session
- `document_id` (UUID): Foreign key to the Document table
- `user_id` (String): Identifier for the user (optional for anonymous sessions)
- `query` (Text): The user's original question/query
- `answer` (Text): The system's response to the query
- `source_chunk_ids` (JSONB): Array of chunk IDs used to generate the answer
- `confidence_score` (Float): Confidence score of the generated answer
- `latency_ms` (Integer): Time taken to generate the answer in milliseconds
- `created_at` (DateTime): Timestamp when conversation entry was created

**Relationships**:
- Many Conversations belong to one Document (many-to-one)
- Many Conversations reference many Chunks through source_chunk_ids (many-to-many via JSONB)

**Validation Rules**:
- `session_id` is required to group conversation entries
- `document_id` must reference a valid Document
- `query` and `answer` are required
- `confidence_score` should be between 0.0 and 1.0

### 5. Query
**Description**: Represents the parameters and context of a search query (conceptual entity for API/processing)

**Fields**:
- `query_text` (String): The text of the user's question
- `document_id` (String): ID of the document to search in
- `session_id` (String): Session identifier
- `user_id` (String): User identifier (optional)
- `top_k` (Integer): Number of chunks to retrieve (default: 5)
- `confidence_threshold` (Float): Minimum confidence for retrieving chunks (default: 0.7)

**Validation Rules**:
- `query_text` must be between 5-500 characters
- `top_k` should be between 1-10
- `confidence_threshold` should be between 0.0-1.0

### 6. Response
**Description**: Represents the system's response to a query (conceptual entity for API)

**Fields**:
- `query` (String): The original user query text
- `answer` (Text): The generated answer from the LLM
- `sources` (Array of Objects): List of source chunks used to generate the answer
- `confidence` (Float): Overall confidence score of the answer
- `latency_ms` (Integer): Time taken to generate the response in milliseconds
- `citations` (String): Formatted citations in [Chapter X, Page Y] format
- `conversation_id` (String): ID of the conversation entry created

**Sources Object Structure**:
- `chunk_id` (String): ID of the source chunk
- `chapter` (Integer): Chapter number of the chunk
- `page` (Integer): Page number of the chunk
- `text` (String): Preview of the source text
- `similarity_score` (Float): Similarity score of this chunk to the query

## Indexes and Performance Considerations

### PostgreSQL Indexes:
1. **documents** table:
   - Index on `book_id` (unique constraint)
   - Index on `ingestion_status`

2. **chunks** table:
   - Index on `document_id` (foreign key relationship)
   - Index on `vector_id` (unique constraint)

3. **conversations** table:
   - Index on `session_id` (to retrieve conversation history)
   - Index on `document_id` (foreign key relationship)

### Vector Database (Qdrant) Collections:
1. **book_embeddings**:
   - Collection for storing all book embeddings with 1536 dimensions
   - Payload schema with chunk_id, document_id, chapter, page, section, text_preview, token_count

2. **selection_*** (dynamic per selection):
   - Dynamic collections for user selections with 1536 dimensions
   - Payload schema with selection_id, document_id, chunk_index, text

## State Transitions

### Document/Book State Transitions:
1. `pending` → `processing` when ingestion starts
2. `processing` → `completed` when ingestion finishes successfully
3. `processing` → `failed` when ingestion encounters an error

### Chunk Creation Process:
1. Document chunks are created during ingestion process
2. Each chunk gets sent to OpenAI for embedding generation
3. Embeddings are stored in Qdrant vector database
4. Chunk metadata with vector_id is stored in PostgreSQL

## Relationships and Referential Integrity

1. **Document-Chunk Relationship**: 
   - Foreign key constraint ensures chunks reference valid documents
   - ON DELETE CASCADE: When a document is deleted, all its chunks are also deleted

2. **Document-Conversation Relationship**:
   - Foreign key constraint ensures conversations reference valid documents
   - ON DELETE CASCADE: When a document is deleted, all associated conversations are deleted

3. **Document-User Selection Relationship**:
   - Foreign key constraint ensures user selections reference valid documents
   - ON DELETE CASCADE: When a document is deleted, all associated user selections are deleted

## Data Validation

All entities have appropriate validation rules enforced at the application and database level:

1. **Input Validation**: All API inputs are validated using Pydantic models
2. **Database Constraints**: Unique, not-null, and foreign key constraints
3. **Business Rules**: Validation of states, relationships, and business logic
4. **Format Validation**: Proper data formats (UUIDs, dates, etc.)

This data model ensures consistency, referential integrity, and performance while supporting all the functional requirements of the RAG Chatbot for Published Books feature.