import uuid
from typing import List, Dict, Optional
from app.services.db_service import DBService
from app.services.embedding_service import embedding_service
from app.services.qdrant_service import qdrant_service
from app.utils.text_processing import text_processing_service
from app.models.database import SessionLocal
import time
import os


class IngestionService:
    def __init__(self):
        self.db_service = DBService
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        self.text_processing_service = text_processing_service

    async def process_document(
        self,
        file_path: str,
        file_type: str,
        book_id: str,
        title: str,
        author: Optional[str] = None,
        publisher: Optional[str] = None,
        metadata: Optional[Dict] = None,
        job_id: Optional[str] = None
    ):
        """Main ingestion pipeline that processes a document for RAG"""
        db = SessionLocal()

        start_time = time.time()
        total_tokens = 0

        try:
            # Initialize DB service
            doc_service = self.db_service(db)

            # Convert metadata to JSON string if it exists
            metadata_str = None
            if metadata:
                import json
                metadata_str = json.dumps(metadata)

            # Create or update document with processing status
            document = doc_service.create_or_update_document(
                book_id=book_id,
                title=title,
                author=author,
                publisher=publisher,
                file_url=file_path,
                metadata=metadata_str
            )

            # Create a job record for tracking
            if job_id:
                doc_service.create_job(
                    job_id=job_id,
                    job_type="ingestion",
                    document_id=document.id,
                    status="processing"
                )

            # Update job progress
            if job_id:
                doc_service.update_job_status(job_id, "processing", progress=10)

            # 2. Extract text from file based on type
            if file_type == 'application/pdf':
                text = await self.text_processing_service.extract_text_from_pdf(file_path)
            elif file_type == 'application/epub+zip':
                text = await self.text_processing_service.extract_text_from_epub(file_path)
            else:
                raise ValueError(f"Unsupported file type: {file_type}")

            # Update job progress
            if job_id:
                doc_service.update_job_status(job_id, "processing", progress=30)

            # 3. Chunk the text
            chunks = await self.text_processing_service.chunk_text(text)

            # Update job progress
            if job_id:
                doc_service.update_job_status(job_id, "processing", progress=50)

            # 4. Generate embeddings for chunks
            chunk_texts = [c["text"] for c in chunks]
            embeddings = await self.embedding_service.embed_batch(chunk_texts)

            # Update job progress
            if job_id:
                doc_service.update_job_status(job_id, "processing", progress=70)

            # 5. Index chunks in Qdrant
            await self.qdrant_service.index_chunks(
                chunks=chunks,
                embeddings=embeddings,
                document_id=str(document.id),  # Use the actual document ID from DB
                document_title=title
            )

            # Update job progress
            if job_id:
                doc_service.update_job_status(job_id, "processing", progress=90)

            # 6. Save chunk metadata to database
            chunk_count = 0
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                chunk_db = doc_service.create_text_chunk(
                    document_id=document.id,
                    chunk_index=i,
                    text=chunk["text"],
                    chapter_num=chunk.get("chapter_num"),
                    section_name=chunk.get("section_name"),
                    page_start=chunk.get("page_start"),
                    page_end=chunk.get("page_end"),
                    token_count=chunk["tokens"],
                    vector_id=None  # Will be set to Qdrant vector ID if available
                )
                chunk_count += 1
                total_tokens += chunk["tokens"]

            # 7. Update document status to completed
            doc_service.update_document_status(str(document.id), 'completed')

            # Update job to completed
            if job_id:
                doc_service.update_job_status(
                    job_id,
                    "completed",
                    progress=100,
                    result=f"Ingestion completed. {chunk_count} chunks created."
                )

            # Calculate duration
            duration = time.time() - start_time

            # Calculate approximate storage used (in MB)
            file_size = os.path.getsize(file_path)
            storage_used_mb = round(file_size / (1024 * 1024), 2)

            return {
                "job_id": job_id,
                "status": "completed",
                "book_id": book_id,
                "document_id": str(document.id),
                "chunks_created": chunk_count,
                "vectors_indexed": chunk_count,  # Assuming 1:1 mapping of chunks to vectors
                "total_tokens": total_tokens,
                "storage_used_mb": storage_used_mb,
                "duration_seconds": round(duration, 2)
            }

        except Exception as e:
            # Update document status to failed
            try:
                if 'doc_service' in locals():
                    doc_service.update_document_status(book_id, 'failed')

                    # Update job status to failed if job_id is provided
                    if job_id:
                        doc_service.update_job_status(
                            job_id,
                            "failed",
                            error_message=str(e)
                        )
            except:
                pass  # If we can't update status, just bubble up the original error
            raise e

        finally:
            db.close()


# Initialize globally
ingestion_service = IngestionService()