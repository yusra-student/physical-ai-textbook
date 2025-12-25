from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from typing import List, Tuple, Optional
from app.config import settings
import uuid


class QdrantService:
    def __init__(self, url: str = None, api_key: str = None):
        """Initialize Qdrant client"""
        self.client = QdrantClient(
            url=url or settings.QDRANT_URL,
            api_key=api_key or settings.QDRANT_API_KEY,
            timeout=30
        )
        self.collection_name = "book_embeddings"
        self.vector_size = 1024  # Cohere embed-english-v3.0 has 1024 dimensions

    async def create_collection_if_not_exists(self):
        """Create collection for 1024-dim Cohere embeddings if it doesn't exist"""
        try:
            # Try to get collection info - this will raise an exception if it doesn't exist
            self.client.get_collection(self.collection_name)
        except:
            # Create collection with 1024-dim vectors and cosine distance
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}' with {self.vector_size}-dim vectors")

    async def index_chunks(
        self,
        chunks: List[dict],
        embeddings: List[List[float]],
        document_id: str,
        document_title: str
    ):
        """Index chunks with 1024-dim Cohere embeddings"""
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = str(uuid.uuid4())
            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "document_id": document_id,
                    "document_title": document_title,
                    "chunk_index": chunk.get("chunk_index", i),
                    "chapter": chunk.get("chapter_num"),
                    "section": chunk.get("section_name"),
                    "text": chunk.get("text", ""),
                    "tokens": chunk.get("token_count"),
                    "page": chunk.get("page_start")
                }
            ))

        # Upsert points to Qdrant collection
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    async def search_similar(
        self,
        query_embedding: List[float],
        document_id: str = None,
        top_k: int = 10,
        score_threshold: float = 0.5
    ) -> List[Tuple[dict, float]]:
        """Search for similar chunks with optional document filtering"""
        query_filter = None
        if document_id:
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="document_id",
                        match=MatchValue(value=document_id)
                    )
                ]
            )

        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            query_filter=query_filter,
            limit=top_k,
            score_threshold=score_threshold
        ).points

        return [(r.payload, r.score) for r in results]

    async def delete_document_chunks(self, document_id: str):
        """Delete all chunks associated with a specific document"""
        # Create filter to find all points with matching document_id
        query_filter = Filter(
            must=[
                FieldCondition(
                    key="document_id",
                    match=MatchValue(value=document_id)
                )
            ]
        )

        # Delete points matching the filter
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(filter=query_filter)
        )

    def health_check(self) -> bool:
        """Check if Qdrant service is accessible"""
        try:
            # Try to get collection info as a health check
            self.client.get_collection(self.collection_name)
            return True
        except:
            return False


# Initialize globally
qdrant_service = QdrantService()