from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from app.models.database import Base
import uuid


class Job(Base):
    __tablename__ = "jobs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    job_id = Column(String(255), unique=True, nullable=False)  # User-facing job ID
    job_type = Column(String(100), nullable=False)  # "ingestion", "query", etc.
    status = Column(String(50), default="pending")  # pending, processing, completed, failed
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id"))  # Optional, for ingestion jobs
    user_id = Column(String(255))  # Optional user associated with the job
    progress = Column(Integer, default=0)  # Progress percentage 0-100
    result = Column(Text)  # Optional result data
    error_message = Column(Text)  # Optional error message if job failed
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    completed_at = Column(DateTime)  # When the job was completed