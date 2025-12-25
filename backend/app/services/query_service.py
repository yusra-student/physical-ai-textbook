from typing import List, Dict, Tuple
import asyncio
from app.services.cohere_service import cohere_service
from app.services.qdrant_service import qdrant_service
from app.services.db_service import DBService
from app.models.database import SessionLocal
from app.models.pydantic_schemas import SourceChunk
from uuid import UUID


class QueryService:
    def __init__(self):
        self.cohere_service = cohere_service
        self.qdrant_service = qdrant_service

    async def query_full_book(
        self,
        query: str,
        document_id: str,
        session_id: str = None,
        user_id: str = None,
        top_k: int = 5,
        confidence_threshold: float = 0.7
    ):
        """
        Main query processing function for full book queries
        """
        db = SessionLocal()
        try:
            # 1. Generate embedding for the query
            query_embedding = await self.cohere_service.embed_text(query)

            # 2. Perform similarity search in Qdrant for the specific document
            search_results = await self.qdrant_service.search_similar(
                query_vector=query_embedding,
                document_id=document_id,
                top_k=top_k
            )

            # 3. Filter results based on confidence threshold
            filtered_results = [
                result for result in search_results 
                if result.get('similarity_score', 0) >= confidence_threshold
            ]

            # 4. Format context for LLM
            context_texts = [result['text'] for result in filtered_results]
            context = "\n\n".join(context_texts)

            # 5. Generate answer using Cohere
            if context.strip():
                answer = await self.cohere_service.generate_response(
                    query=query,
                    context=context
                )
            else:
                answer = "I couldn't find information about this in the book. Please try rephrasing your question or check another document."

            # 6. Calculate confidence score
            confidence = self._calculate_confidence(filtered_results)

            # 7. Format sources
            sources = self._format_sources(filtered_results)

            # 8. Create citations
            citations = self._format_citations(filtered_results)

            # 9. Save conversation entry to database
            db_service = DBService(db)
            conversation = db_service.create_conversation_entry(
                session_id=session_id or f"session_{document_id}",
                document_id=UUID(document_id),
                query=query,
                answer=answer,
                user_id=user_id,
                source_chunk_ids=str([r.get('chunk_id') for r in filtered_results]),
                confidence_score=confidence,
                latency_ms=0  # Placeholder - would calculate actual time
            )

            # 10. Return formatted response
            return {
                "query": query,
                "answer": answer,
                "sources": sources,
                "confidence": confidence,
                "latency_ms": 0,  # Placeholder
                "citations": citations,
                "conversation_id": str(conversation.id)
            }

        except Exception as e:
            raise e
        finally:
            db.close()

    def _calculate_confidence(self, search_results: List[Dict]) -> float:
        """
        Calculate overall confidence based on similarity scores
        """
        if not search_results:
            return 0.0

        avg_similarity = sum(r.get('similarity_score', 0) for r in search_results) / len(search_results)
        return avg_similarity

    def _format_sources(self, search_results: List[Dict]) -> List[SourceChunk]:
        """
        Format search results into SourceChunk objects
        """
        sources = []
        for result in search_results:
            source = SourceChunk(
                chunk_id=result.get('chunk_id', ''),
                chapter=result.get('chapter_num'),
                page=result.get('page_num'),
                text=result.get('text', '')[:200] + "..." if len(result.get('text', '')) > 200 else result.get('text', ''),
                similarity_score=result.get('similarity_score', 0.0)
            )
            sources.append(source)
        return sources

    def _format_citations(self, search_results: List[Dict]) -> str:
        """
        Format citations from search results
        """
        citations = []
        for result in search_results:
            chapter = result.get('chapter_num')
            page = result.get('page_num')
            if chapter and page:
                citations.append(f"[Chapter {chapter}, Page {page}]")
            elif page:
                citations.append(f"[Page {page}]")
            elif chapter:
                citations.append(f"[Chapter {chapter}]")
        
        return " and ".join(citations) if citations else "No specific citations"


# Initialize globally
query_service = QueryService()