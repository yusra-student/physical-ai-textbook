import os
from dotenv import load_dotenv
from flask import Flask, request, jsonify
from flask_cors import CORS
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Distance, FieldCondition, Filter, MatchValue
from openai import OpenAI

# Load environment variables from .env file
load_dotenv()

app = Flask(__name__)
CORS(app)  # Enable CORS for all origins

# --- CONFIGURATION ---
COLLECTION_NAME = "ai_book_rag" # Should match the one in ingest.py

# Cohere API Key for embeddings
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY not found in .env file")
cohere_client = cohere.Client(COHERE_API_KEY)
EMBED_MODEL = "embed-english-v3.0"  # Cohere embedding model

# Qdrant Cloud configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
if not QDRANT_URL or not QDRANT_API_KEY:
    raise ValueError("QDRANT_URL and QDRANT_API_KEY not found in .env file")
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Gemini API for LLM
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY not found in .env file")

# Initialize OpenAI client for Gemini (using OpenAI-compatible endpoint)
gemini_client = OpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

LLM_MODEL = "gemini-1.5-flash" # Use the correct model ID for Gemini

# --- EMBEDDINGS ---
def get_embedding(text: str):
    """Generates embedding for the given text using Cohere Embed v3."""
    response = cohere_client.embed(
        texts=[text],
        model=EMBED_MODEL,
        input_type="search_query",
    )
    return response.embeddings[0]

# --- RETRIEVAL ---
def retrieve_context(query: str, top_k: int = 5) -> str:
    """Retrieves relevant text chunks from Qdrant based on the query."""
    # The error indicates we should use query_text directly, as the client handles embedding.
    search_result = qdrant_client.query(
        collection_name=COLLECTION_NAME,
        query_text=query, # Use query_text instead of query_vector
        limit=top_k,
        with_payload=True,
    )

    context = ""
    # The `query` method returns a tuple, where the first element is the list of points
    points = search_result[0] if isinstance(search_result, tuple) else search_result
    for hit in points:
        context += hit.payload.get("text", "") + "\n\n"
    return context.strip()

# --- LLM RESPONSE GENERATION ---
def generate_response(user_query: str, context: str) -> str:
    """Generates a response using the Gemini LLM based on user query and retrieved context."""
    
    # Simple prompt engineering with context
    messages = [
        {"role": "system", "content": "You are an AI assistant that answers questions based on the provided context only. If the answer is not in the context, say 'I don't have enough information to answer this question from the provided context.' Do not make up answers."},
        {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {user_query}"}
    ]

    try:
        chat_completion = gemini_client.chat.completions.create(
            model=LLM_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=500,
        )
        return chat_completion.choices[0].message.content
    except Exception as e:
        print(f"Error calling Gemini LLM: {e}")
        return "I am currently unable to generate a response. Please try again later."

# --- FLASK API ENDPOINT ---
@app.route("/chat", methods=["POST"])
def chat():
    user_message = request.json.get("message")
    if not user_message:
        return jsonify({"error": "No message provided"}), 400

    print(f"Received message: {user_message}")

    # 1. Retrieve relevant context
    context = retrieve_context(user_message)

    # 2. Generate response using LLM
    response_content = generate_response(user_message, context)

    return jsonify({"response": response_content})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
