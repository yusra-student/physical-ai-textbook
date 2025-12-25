from app.services.cohere_service import CohereService
from typing import List
from app.config import settings


class EmbeddingService:
    def __init__(self):
        self.service = CohereService(settings.COHERE_API_KEY)

    async def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """Embed texts for indexing"""
        return self.service.embed_documents(texts)

    async def embed_query(self, query: str) -> List[float]:
        """Embed query for search"""
        return self.service.embed_query(query)

    async def embed_batch(
        self, texts: List[str], batch_size: int = 96  # Cohere API limit
    ) -> List[List[float]]:
        """Embed large number of texts in batches"""
        all_embeddings = []
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = await self.embed_texts(batch)
            all_embeddings.extend(batch_embeddings)
        return all_embeddings

    def get_dimension(self) -> int:
        """Get embedding dimension"""
        return self.service.get_embedding_dimension()

    def health_check(self) -> bool:
        """Check if embedding service is accessible"""
        return self.service.health_check()


# Initialize globally
embedding_service = EmbeddingService()