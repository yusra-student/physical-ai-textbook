from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid

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