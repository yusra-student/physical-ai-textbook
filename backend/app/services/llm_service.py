from app.services.cohere_service import CohereService
from typing import List
from app.config import settings


class LLMService:
    def __init__(self):
        self.service = CohereService()

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[str],
        document_title: str = None,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> str:
        """
        Generate answer using Cohere Command model with context
        """
        try:
            # Build system message
            system_message = f"""
            You are a helpful assistant answering questions about the book: '{document_title or 'a published book'}'.

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
            {query}

            ANSWER:
            """

            response = self.service.client.generate(
                model=self.service.llm_model,
                prompt=full_prompt,
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.generations[0].text.strip()
        except Exception as e:
            return f"Error generating response: {str(e)}"

    def health_check(self) -> bool:
        """Check if LLM service is accessible"""
        return self.service.health_check()


# Initialize globally
llm_service = LLMService()