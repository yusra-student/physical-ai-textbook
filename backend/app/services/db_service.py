from sqlalchemy.orm import Session
from app.models.document import Document
from app.models.chunk import TextChunk
from app.models.user_selection import UserSelection
from app.models.conversation import Conversation
from app.models.job import Job
from app.models.database import SessionLocal
from typing import List, Optional
from uuid import UUID


class DBService:
    def __init__(self, db: Session):
        self.db = db

    # Document operations
    def create_document(self, book_id: str, title: str, author: str = None,
                       publisher: str = None, file_url: str = None, metadata: str = None) -> Document:
        document = Document(
            book_id=book_id,
            title=title,
            author=author,
            publisher=publisher,
            file_url=file_url,
            metadata=metadata
        )
        self.db.add(document)
        self.db.commit()
        self.db.refresh(document)
        return document

    def get_document_by_id(self, document_id: UUID) -> Document:
        return self.db.query(Document).filter(Document.id == document_id).first()

    def get_document_by_book_id(self, book_id: str) -> Document:
        return self.db.query(Document).filter(Document.book_id == book_id).first()

    def update_document_status(self, book_id: str, status: str) -> Document:
        document = self.db.query(Document).filter(Document.book_id == book_id).first()
        if document:
            document.ingestion_status = status
            self.db.commit()
            self.db.refresh(document)
        return document

    def create_or_update_document(self, book_id: str, title: str, author: str = None,
                                  publisher: str = None, file_url: str = None,
                                  metadata: str = None) -> Document:
        # Check if document with book_id already exists
        document = self.db.query(Document).filter(Document.book_id == book_id).first()
        if document:
            # Update existing document
            document.title = title
            document.author = author
            document.publisher = publisher
            document.file_url = file_url
            document.metadata = metadata
            document.ingestion_status = 'processing'  # Reset status when updating
        else:
            # Create new document
            document = Document(
                book_id=book_id,
                title=title,
                author=author,
                publisher=publisher,
                file_url=file_url,
                metadata=metadata,
                ingestion_status='processing'
            )
            self.db.add(document)

        self.db.commit()
        self.db.refresh(document)
        return document

    # Text chunk operations
    def create_text_chunk(self, document_id: UUID, chunk_index: int, text: str,
                         chapter_num: int = None, section_name: str = None,
                         page_start: int = None, page_end: int = None,
                         token_count: int = None, vector_id: str = None) -> TextChunk:
        chunk = TextChunk(
            document_id=document_id,
            chunk_index=chunk_index,
            text=text,
            chapter_num=chapter_num,
            section_name=section_name,
            page_start=page_start,
            page_end=page_end,
            token_count=token_count,
            vector_id=vector_id
        )
        self.db.add(chunk)
        self.db.commit()
        self.db.refresh(chunk)
        return chunk

    def get_chunks_by_document(self, document_id: UUID) -> List[TextChunk]:
        return self.db.query(TextChunk).filter(TextChunk.document_id == document_id).all()

    # User selection operations
    def create_user_selection(self, document_id: UUID, selected_text: str,
                             session_id: str = None, start_position: int = None,
                             end_position: int = None,
                             qdrant_namespace: str = None) -> UserSelection:
        selection = UserSelection(
            document_id=document_id,
            selected_text=selected_text,
            session_id=session_id,
            start_position=start_position,
            end_position=end_position,
            qdrant_namespace=qdrant_namespace
        )
        self.db.add(selection)
        self.db.commit()
        self.db.refresh(selection)
        return selection

    def get_selection_by_id(self, selection_id: UUID) -> UserSelection:
        return self.db.query(UserSelection).filter(UserSelection.id == selection_id).first()

    # Conversation operations
    def create_conversation_entry(self, session_id: str, document_id: UUID, query: str,
                                 answer: str, user_id: str = None,
                                 source_chunk_ids: str = None,
                                 confidence_score: float = None,
                                 latency_ms: int = None) -> Conversation:
        conversation = Conversation(
            session_id=session_id,
            document_id=document_id,
            user_id=user_id,
            query=query,
            answer=answer,
            source_chunk_ids=source_chunk_ids,
            confidence_score=confidence_score,
            latency_ms=latency_ms
        )
        self.db.add(conversation)
        self.db.commit()
        self.db.refresh(conversation)
        return conversation

    def get_conversation_by_session(self, session_id: str) -> List[Conversation]:
        return self.db.query(Conversation).filter(Conversation.session_id == session_id).all()

    # Job operations for tracking long-running tasks
    def create_job(self, job_id: str, job_type: str, document_id: UUID = None,
                   user_id: str = None, status: str = 'pending') -> Job:
        job = Job(
            job_id=job_id,
            job_type=job_type,
            status=status,
            document_id=document_id,
            user_id=user_id
        )
        self.db.add(job)
        self.db.commit()
        self.db.refresh(job)
        return job

    def get_job_by_id(self, job_id: str) -> Job:
        return self.db.query(Job).filter(Job.job_id == job_id).first()

    def update_job_status(self, job_id: str, status: str, progress: int = None,
                         error_message: str = None, result: str = None) -> Job:
        job = self.db.query(Job).filter(Job.job_id == job_id).first()
        if job:
            job.status = status
            if progress is not None:
                job.progress = progress
            if error_message:
                job.error_message = error_message
            if result:
                job.result = result
            if status in ['completed', 'failed']:
                from sqlalchemy.sql import func
                job.completed_at = func.now()
            self.db.commit()
            self.db.refresh(job)
        return job

    def get_job_status(self, job_id: str) -> Job:
        return self.db.query(Job).filter(Job.job_id == job_id).first()


# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Initialize DB service
db_service = DBService