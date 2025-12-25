from fastapi import APIRouter, HTTPException, Depends
import time
from app.models.database import get_db
from app.models.pydantic_schemas import QueryRequest, QueryResponse, SourceChunk
from app.models.document import Document
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.services.db_service import db_service
from app.services.llm_service import llm_service
from sqlalchemy.orm import Session

router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_full_book(
    request: QueryRequest,
    db: Session = Depends(get_db)
):
    """
    Ask questions about the content of a specific book
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
        sources = []
        for payload, score in search_results:
            chunk_text = payload.get("text", "")
            if chunk_text:
                context_chunks.append(chunk_text)

                # Create source object for response
                source = SourceChunk(
                    chunk_id=payload.get("chunk_id", ""),
                    chapter=payload.get("chapter"),
                    page=payload.get("page"),
                    text=chunk_text[:200] + "..." if len(chunk_text) > 200 else chunk_text,  # Preview
                    similarity_score=score
                )
                sources.append(source)

        if not context_chunks:
            return QueryResponse(
                query=request.query,
                answer="I couldn't find relevant information in the book to answer this question.",
                sources=[],
                confidence=0.0,
                latency_ms=int((time.time() - start_time) * 1000),
                citations="",
                conversation_id=""
            )

        # 4. Generate answer using LLM service
        answer = await llm_service.generate_answer(
            query=request.query,
            context_chunks=context_chunks,
            document_title=document.title
        )

        # 5. Calculate confidence score (average of similarity scores)
        avg_similarity = sum([s.similarity_score for s in sources]) / len(sources) if sources else 0.0
        confidence = min(avg_similarity, 1.0)  # Ensure confidence is not above 1.0

        # 6. Extract citations
        citations = []
        for source in sources:
            if source.chapter is not None and source.page is not None:
                citations.append(f"[Chapter {source.chapter}, Page {source.page}]")
            elif source.chapter is not None:
                citations.append(f"[Chapter {source.chapter}]")
            elif source.page is not None:
                citations.append(f"[Page {source.page}]")

        citations_str = " and ".join(citations) if citations else "No specific citations available"

        # 7. Create conversation entry
        db_session = db_service(db)
        conversation = db_session.create_conversation_entry(
            session_id=request.session_id or "anonymous",
            document_id=request.document_id,
            query=request.query,
            answer=answer,
            user_id=request.user_id,
            source_chunk_ids=str([s.chunk_id for s in sources]),  # Convert list to string for storage
            confidence_score=confidence,
            latency_ms=int((time.time() - start_time) * 1000)
        )

        # 8. Return formatted response
        return QueryResponse(
            query=request.query,
            answer=answer,
            sources=sources,
            confidence=confidence,
            latency_ms=int((time.time() - start_time) * 1000),
            citations=citations_str,
            conversation_id=str(conversation.id)
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error during query processing",
            "message": str(e),
            "code": "SERVER_ERROR"
        })