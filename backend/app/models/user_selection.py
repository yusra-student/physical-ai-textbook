from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid

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