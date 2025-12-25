import cohere
from typing import List, Optional
from app.config import settings


class CohereService:
    def __init__(self, api_key: str = None):
        """Initialize Cohere client"""
        self.client = cohere.Client(api_key=api_key or settings.COHERE_API_KEY)
        self.embed_model = settings.COHERE_EMBED_MODEL
        self.llm_model = settings.COHERE_LLM_MODEL
        self.rerank_model = settings.COHERE_RERANK_MODEL

    # ===== EMBEDDING METHODS =====
    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for texts
        Args:
            texts: List of texts to embed
            input_type: "search_document", "search_query", "classification", etc.
        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embed(
                model=self.embed_model,
                texts=texts,
                input_type=input_type
            )
            return response.embeddings
        except Exception as e:
            raise Exception(f"Embedding failed: {str(e)}")

    def embed_query(self, query: str) -> List[float]:
        """Embed search query (special input type)"""
        return self.embed_texts([query], input_type="search_query")[0]

    def embed_documents(self, documents: List[str]) -> List[List[float]]:
        """Embed documents for indexing"""
        return self.embed_texts(documents, input_type="search_document")

    # ===== GENERATION METHODS =====
    def generate_answer(
        self, 
        prompt: str, 
        context_chunks: List[str], 
        max_tokens: int = 500, 
        temperature: float = 0.7
    ) -> str:
        """
        Generate answer using Cohere Command model
        Args:
            prompt: User question
            context_chunks: Book excerpts for context
            max_tokens: Max response length
            temperature: Creativity (0-1)
        Returns:
            Generated answer
        """
        # Build system message
        system_message = """
        You are a helpful assistant answering questions about a published book.
        
        RULES:
        1. Answer ONLY based on the provided book excerpts
        2. If information is not in the excerpts, say "I couldn't find this in the book"
        3. Always cite sources with [Chapter X, Page Y] when available
        4. Be concise and clear
        5. Do not add external knowledge or speculation
        """

        # Build context
        context_text = "\n\n".join([
            f"[Excerpt {i+1}]\n{chunk}" 
            for i, chunk in enumerate(context_chunks)
        ])

        # Build full prompt
        full_prompt = f"""
        {system_message}
        
        CONTEXT FROM BOOK:
        {context_text}
        
        QUESTION:
        {prompt}
        
        ANSWER:
        """

        try:
            response = self.client.generate(
                model=self.llm_model,
                prompt=full_prompt,
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.generations[0].text.strip()
        except Exception as e:
            return f"Error generating response: {str(e)}"

    # ===== RE-RANKING METHODS (OPTIONAL) =====
    def rerank_results(self, query: str, documents: List[str], top_k: int = 5) -> List[dict]:
        """
        Re-rank documents for better relevance
        This is optional but improves precision when:
        - You have 20+ candidate documents
        - Precision is critical
        Cost: $2 per 1K queries (enable for production)
        """
        try:
            response = self.client.rerank(
                model=self.rerank_model,
                query=query,
                documents=documents,
                top_n=top_k
            )
            
            return [
                {
                    "index": result.index,
                    "relevance_score": result.relevance_score,
                    "document": result.document
                }
                for result in response.results
            ]
        except Exception as e:
            # If reranking fails, return top-k by index
            return [
                {
                    "index": i,
                    "relevance_score": 1.0 - i*0.1,
                    "document": doc
                }
                for i, doc in enumerate(documents[:top_k])
            ]

    # ===== UTILITY METHODS =====
    def get_embedding_dimension(self) -> int:
        """Get embedding dimension (1024 for embed-english-v3.0)"""
        return 1024  # Cohere embed-english-v3.0 has 1024 dimensions

    def health_check(self) -> bool:
        """Check if Cohere API is accessible"""
        try:
            # Simple test call
            self.embed_query("test")
            return True
        except:
            return False


# Initialize globally
cohere_service = CohereService()