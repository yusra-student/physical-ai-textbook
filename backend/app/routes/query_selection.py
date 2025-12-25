from fastapi import APIRouter, HTTPException, Depends
import time
from app.models.database import get_db
from app.models.pydantic_schemas import (
    SelectionQueryRequest, SelectionQueryResponse, SelectionSourceChunk
)
from app.models.document import Document
from app.models.user_selection import UserSelection
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.services.db_service import db_service
from app.services.llm_service import llm_service
from sqlalchemy.orm import Session

router = APIRouter()


@router.post("/query-selection")
async def query_selection(
    request: SelectionQueryRequest,
    db: Session = Depends(get_db)
):
    """
    Answer questions based only on a specific text selection made by the user
    """
    # Validate selection exists in the database
    selection = db.query(UserSelection).filter(UserSelection.id == request.selection_id).first()
    if not selection:
        raise HTTPException(status_code=404, detail={
            "error": "Selection not found",
            "message": f"Selection with ID {request.selection_id} not found",
            "code": "SELECTION_NOT_FOUND"
        })

    # Validate document exists
    document = db.query(Document).filter(Document.id == request.document_id).first()
    if not document:
        raise HTTPException(status_code=404, detail={
            "error": "Book not found",
            "message": f"Document with ID {request.document_id} not found",
            "code": "DOCUMENT_NOT_FOUND"
        })

    start_time = time.time()

    try:
        # 1. Generate embedding for query
        query_embedding = await embedding_service.embed_query(request.query)

        # 2. Perform similarity search in Qdrant for the selection-specific namespace
        # For user selections, we might have a specific namespace or just use the selected text directly
        # Since this is a direct selection query, we'll use the selected text as context
        selected_text = request.selected_text

        # 3. Format context for LLM (restricted to selection)
        context_chunks = [selected_text]

        # Create source object for the selected text
        source = SelectionSourceChunk(
            selection_id=request.selection_id,
            position="user-selected",
            text=selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
            similarity_score=1.0  # Perfect match since it's the exact selection
        )
        sources = [source]

        # 4. Generate answer using LLM service, restricted to selection
        answer = await llm_service.generate_answer(
            query=request.query,
            context_chunks=context_chunks,
            document_title=document.title,
            max_tokens=500,
            temperature=0.3  # Lower temperature for more focused answers
        )

        # 5. Create conversation entry
        db_session = db_service(db)
        conversation = db_session.create_conversation_entry(
            session_id=request.session_id or "anonymous",
            document_id=request.document_id,
            query=request.query,
            answer=answer,
            user_id=None,  # Selection queries might not have user_id
            source_chunk_ids=str([request.selection_id]),  # Reference to the selection
            confidence_score=1.0,  # Using the selection as context
            latency_ms=int((time.time() - start_time) * 1000)
        )

        # 6. Return formatted response
        return SelectionQueryResponse(
            query=request.query,
            answer=answer,
            sources=sources,
            scope="restricted_to_selection",
            citation="[From your selected text]",
            conversation_id=str(conversation.id)
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error during selection query processing",
            "message": str(e),
            "code": "SERVER_ERROR"
        })