from fastapi import APIRouter, HTTPException, Depends
import time
from app.models.database import get_db
from app.models.pydantic_schemas import QueryRequest, QueryResponse
from app.models.document import Document
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.services.db_service import db_service
from app.services.llm_service import llm_service
from sqlalchemy.orm import Session

router = APIRouter()


@router.post("/ask")
async def ask_question(
    request: QueryRequest,
    db: Session = Depends(get_db)
):
    """
    Ask questions about the content of a specific book - compatible with frontend
    """
    # Validate that document exists
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

        # 2. Perform similarity search in Qdrant for the specific document
        search_results = await qdrant_service.search_similar(
            query_embedding=query_embedding,
            document_id=request.document_id,
            top_k=request.top_k or 5,
            score_threshold=request.confidence_threshold or 0.5
        )

        # 3. Format context for LLM and extract sources
        context_chunks = []
        context = []
        for payload, score in search_results:
            chunk_text = payload.get("text", "")
            if chunk_text:
                context_chunks.append(chunk_text)
                
                # Format context in the way frontend expects
                context.append({
                    "text": chunk_text,
                    "score": score,
                    "chunk_id": payload.get("chunk_id", ""),
                    "chapter": payload.get("chapter"),
                    "page": payload.get("page")
                })

        if not context_chunks:
            return {
                "response": "I couldn't find relevant information in the book to answer this question.",
                "context": [],
                "query": request.query,
                "answer": "I couldn't find relevant information in the book to answer this question.",
                "sources": [],
                "confidence": 0.0,
                "latency_ms": int((time.time() - start_time) * 1000)
            }

        # 4. Generate answer using LLM service
        answer = await llm_service.generate_answer(
            query=request.query,
            context_chunks=context_chunks,
            document_title=document.title
        )

        # 5. Calculate confidence score (average of similarity scores)
        avg_similarity = sum([ctx["score"] for ctx in context]) / len(context) if context else 0.0
        confidence = min(avg_similarity, 1.0)  # Ensure confidence is not above 1.0

        # 6. Create conversation entry
        db_session = db_service(db)
        conversation = db_session.create_conversation_entry(
            session_id=request.session_id or "anonymous",
            document_id=request.document_id,
            query=request.query,
            answer=answer,
            user_id=request.user_id,
            source_chunk_ids=str([ctx["chunk_id"] for ctx in context]),  # Convert list to string for storage
            confidence_score=confidence,
            latency_ms=int((time.time() - start_time) * 1000)
        )

        # 7. Return formatted response compatible with frontend
        return {
            "response": answer,
            "context": context,
            "query": request.query,
            "answer": answer,
            "sources": context,  # Also include as sources for compatibility
            "confidence": confidence,
            "latency_ms": int((time.time() - start_time) * 1000),
            "conversation_id": str(conversation.id)
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error during query processing",
            "message": str(e),
            "code": "SERVER_ERROR"
        })