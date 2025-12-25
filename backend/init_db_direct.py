import os
from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, ForeignKey, Enum
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import uuid

# Create the database engine with the correct URL
DATABASE_URL = "postgresql://postgres:postgres@localhost:5432/rag_chatbot"
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Define the Document model directly in this script
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
    document_metadata = Column(Text)  # JSON metadata stored as text

# Define the TextChunk model directly in this script
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

def create_tables():
    """Create all database tables"""
    print("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully!")

if __name__ == "__main__":
    create_tables()