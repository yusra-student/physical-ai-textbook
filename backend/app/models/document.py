from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, Enum
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid

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