from sqlalchemy import Column, String, Text, DateTime, Integer, Float, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid

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