from pydantic import BaseModel
from typing import List, Optional
from uuid import UUID


class SourceChunk(BaseModel):
    chunk_id: str
    chapter: Optional[int] = None
    page: Optional[int] = None
    text: str
    similarity_score: float


class QueryRequest(BaseModel):
    query: str
    document_id: str
    session_id: Optional[str] = None
    user_id: Optional[str] = None
    top_k: Optional[int] = 5
    confidence_threshold: Optional[float] = 0.7


class QueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[SourceChunk]
    confidence: float
    latency_ms: int
    citations: str
    conversation_id: str


class SelectionQueryRequest(BaseModel):
    query: str
    selection_id: str
    document_id: str
    selected_text: str
    session_id: Optional[str] = None


class SelectionSourceChunk(BaseModel):
    selection_id: str
    position: str
    text: str
    similarity_score: float


class SelectionQueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[SelectionSourceChunk]
    scope: str
    citation: str
    conversation_id: str


class CreateConversationRequest(BaseModel):
    document_id: str
    user_id: Optional[str] = None
    metadata: Optional[dict] = None


class CreateConversationResponse(BaseModel):
    session_id: str
    document_id: str
    created_at: str
    ttl_hours: int = 24


class Message(BaseModel):
    id: str
    type: str
    content: str
    sources: Optional[List[SourceChunk]] = None
    timestamp: str


class ConversationResponse(BaseModel):
    session_id: str
    document_id: str
    message_count: int
    messages: List[Message]


class ErrorResponse(BaseModel):
    error: str
    message: str
    code: str