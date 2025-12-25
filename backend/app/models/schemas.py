from sqlalchemy import Column, Integer, String, DateTime, Text, Float, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid
from sqlalchemy import Enum


class Document(Base):
    __tablename__ = "documents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    book_id = Column(String(255), unique=True, nullable=False)
    title = Column(String(500), nullable=False)
    author = Column(String(500))
    publisher = Column(String(500))
    published_date = Column(DateTime)
    total_pages = Column(Integer)
    file_url = Column(Text)
    ingestion_status = Column(Enum('pending', 'processing', 'completed', 'failed', name='ingestion_status_enum'), default='pending')
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    metadata = Column(Text)  # JSON metadata stored as text


class TextChunk(Base):
    __tablename__ = "chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id"), nullable=False)
    chunk_index = Column(Integer)
    text = Column(Text, nullable=False)
    chapter_num = Column(Integer)
    section_name = Column(String(500))
    page_start = Column(Integer)
    page_end = Column(Integer)
    token_count = Column(Integer)
    vector_id = Column(String(255), unique=True)  # Unique vector ID in Qdrant
    created_at = Column(DateTime, default=func.now())


class UserSelection(Base):
    __tablename__ = "user_selections"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id"), nullable=False)
    session_id = Column(String(255))
    selected_text = Column(Text, nullable=False)
    start_position = Column(Integer)
    end_position = Column(Integer)
    qdrant_namespace = Column(String(255), unique=True)  # Unique namespace in Qdrant for this selection
    created_at = Column(DateTime, default=func.now())


class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), nullable=False)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id"), nullable=False)
    user_id = Column(String(255))
    query = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    source_chunk_ids = Column(Text)  # JSON array of chunk IDs as text
    confidence_score = Column(Float)
    latency_ms = Column(Integer)
    created_at = Column(DateTime, default=func.now())


class QueryResponse(Base):
    __tablename__ = "query_responses"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_text = Column(Text, nullable=False)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id"), nullable=False)
    session_id = Column(String(255))
    user_id = Column(String(255))
    response_text = Column(Text, nullable=False)
    sources = Column(Text)  # JSON of source information
    confidence_score = Column(Float)
    created_at = Column(DateTime, default=func.now())