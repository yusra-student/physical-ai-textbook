from fastapi import APIRouter, UploadFile, File, Form, HTTPException, Depends
from typing import Optional
from app.services.ingest_service import ingestion_service
from app.models.database import get_db
from app.models.document import Document
from app.models.chunk import TextChunk
from app.models.job import Job
from sqlalchemy.orm import Session
import uuid
import json

router = APIRouter()


@router.post("/ingest", status_code=202)
async def ingest_book(
    file: UploadFile = File(...),
    book_id: str = Form(...),
    title: str = Form(...),
    author: Optional[str] = Form(None),
    publisher: Optional[str] = Form(None),
    metadata: Optional[str] = Form(None),  # JSON string
    db: Session = Depends(get_db)
):
    """
    Upload and process a book (PDF/EPUB) for RAG
    """
    # Validate file type
    if file.content_type not in ["application/pdf", "application/epub+zip"]:
        raise HTTPException(
            status_code=400,
            detail={
                "error": "Invalid file format",
                "message": "Only PDF and EPUB files are supported",
                "code": "INVALID_FILE_FORMAT"
            }
        )

    # Check file size (500MB limit)
    # Read file to check its size
    file_content = await file.read()

    # Reset file pointer to beginning for later processing
    await file.seek(0)

    if len(file_content) > 500 * 1024 * 1024:  # 500MB in bytes
        raise HTTPException(
            status_code=413,
            detail={
                "error": "File too large",
                "message": "File size exceeds 500MB limit",
                "code": "FILE_TOO_LARGE"
            }
        )

    # Parse metadata JSON if provided
    metadata_dict = None
    if metadata:
        try:
            metadata_dict = json.loads(metadata)
        except json.JSONDecodeError:
            raise HTTPException(
                status_code=400,
                detail={
                    "error": "Invalid metadata format",
                    "message": "Metadata must be valid JSON",
                    "code": "INVALID_METADATA"
                }
            )

    # Generate job_id for tracking
    job_id = str(uuid.uuid4())

    try:
        # Save uploaded file temporarily (in a real implementation, you might want to use a temp directory)
        file_location = f"temp_{job_id}_{file.filename}"
        with open(file_location, "wb+") as file_object:
            # file.read() was already called earlier, so file_content contains the actual file data
            file_object.write(file_content)

        # Process the document asynchronously
        result = await ingestion_service.process_document(
            file_path=file_location,
            file_type=file.content_type,
            book_id=book_id,
            title=title,
            author=author,
            publisher=publisher,
            metadata=metadata_dict,
            job_id=job_id
        )

        # Prepare response matching the API specification
        return {
            "job_id": job_id,
            "status": result["status"],
            "book_id": book_id,
            "estimated_completion": "2024-12-17T15:45:00Z",  # Placeholder
            "webhook_url": None  # Optional webhook URL for notifications
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error during ingestion",
            "message": str(e),
            "code": "SERVER_ERROR"
        })


@router.get("/ingest/status/{job_id}")
async def get_ingestion_status(
    job_id: str,
    db: Session = Depends(get_db)
):
    """
    Check the status of a book ingestion job
    """
    try:
        # Query the job based on job_id from the jobs table
        job = db.query(Job).filter(Job.job_id == job_id).first()

        if not job:
            raise HTTPException(status_code=404, detail={
                "error": "Job not found",
                "message": f"Ingestion job with ID {job_id} not found",
                "code": "JOB_NOT_FOUND"
            })

        # If the job is associated with a document, get document statistics
        chunks_created = 0
        total_tokens = 0
        document_id = str(job.document_id) if job.document_id else None

        if job.document_id:
            # Count the number of chunks associated with this document
            chunks_created = db.query(TextChunk).filter(TextChunk.document_id == job.document_id).count()

            # For now, we'll assume vectors_indexed equals chunks_created
            # In a more sophisticated implementation, this might track separately
            vectors_indexed = chunks_created

            # Calculate total tokens from text chunks
            if chunks_created > 0:
                all_chunks = db.query(TextChunk).filter(TextChunk.document_id == job.document_id).all()
                for chunk in all_chunks:
                    if chunk.token_count:
                        total_tokens += chunk.token_count
        else:
            vectors_indexed = chunks_created

        # Return status response matching API specification
        return {
            "job_id": job_id,
            "status": job.status,
            "document_id": document_id,
            "chunks_created": chunks_created,
            "vectors_indexed": vectors_indexed,
            "total_tokens": total_tokens,
            "storage_used_mb": 0.0,  # Placeholder - would come from actual ingestion tracking
            "duration_seconds": 0,  # Placeholder - would come from actual ingestion tracking
            "progress": job.progress or 0,
            "error_message": job.error_message
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error retrieving status",
            "message": str(e),
            "code": "SERVER_ERROR"
        })