import os
import requests
import xml.etree.ElementTree as ET
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

from langchain_community.document_loaders import DirectoryLoader, UnstructuredMarkdownLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter

# Load environment variables
load_dotenv()

# --- CONFIGURATION ---
DOCS_PATH = "../ai-book/docs"  # Path to your Docusaurus docs
COLLECTION_NAME = "ai_book_rag" # Unique collection name for Qdrant

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


# --- STEP 1: LOAD DOCUMENTS ---
def load_docs(directory_path):
    print(f"Loading documents from {directory_path}...")
    # Using DirectoryLoader with UnstructuredMarkdownLoader for markdown files
    loader = DirectoryLoader(
        directory_path,
        glob="**/*.md*",  # Matches both .md and .mdx files
        loader_cls=UnstructuredMarkdownLoader,
        recursive=True
    )
    documents = loader.load()
    print(f"Loaded {len(documents)} documents.")
    return documents

# --- STEP 2: CHUNK DOCUMENTS ---
def chunk_docs(documents):
    print("Chunking documents...")
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        length_function=len,
        is_separator_regex=False,
    )
    chunks = text_splitter.split_documents(documents)
    print(f"Created {len(chunks)} chunks.")
    return chunks

# --- STEP 3: CREATE EMBEDDINGS ---
def get_embeddings(texts):
    print(f"Creating embeddings for {len(texts)} texts using Cohere...")
    response = cohere_client.embed(
        texts=texts,
        model=EMBED_MODEL,
        input_type="classification" # or "clustering" or "search_document" or "search_query"
    )
    return response.embeddings

# --- STEP 4: STORE IN QDRANT ---
def create_qdrant_collection():
    print(f"Recreating Qdrant collection '{COLLECTION_NAME}'...")
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,  # Cohere embed-english-v3.0 dimension
            distance=Distance.COSINE
        )
    )
    print("Collection created.")

def store_chunks_in_qdrant(chunks):
    print("Storing chunks in Qdrant...")
    # Extract text content and generate embeddings in batches
    texts = [chunk.page_content for chunk in chunks]
    metadatas = [chunk.metadata for chunk in chunks]

    # Batching embeddings creation to optimize API calls
    batch_size = 96  # Cohere limit
    all_embeddings = []
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        batch_embeddings = get_embeddings(batch_texts)
        all_embeddings.extend(batch_embeddings)
        print(f"Processed embeddings for batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

    points = []
    for i, chunk in enumerate(chunks):
        # Ensure metadata is serializable (e.g., convert Path objects to strings)
        cleaned_metadata = {k: str(v) for k, v in metadatas[i].items()}
        points.append(
            PointStruct(
                id=i,  # Unique ID for each chunk
                vector=all_embeddings[i],
                payload={
                    "text": chunk.page_content,
                    "source": cleaned_metadata.get("source", "unknown"),
                    **cleaned_metadata # Add all cleaned metadata
                }
            )
        )
    
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points,
        wait=True
    )
    print(f"Stored {len(points)} points in Qdrant.")

# --- MAIN INGESTION PROCESS ---
def ingest_documents():
    documents = load_docs(DOCS_PATH)
    chunks = chunk_docs(documents)
    create_qdrant_collection()
    store_chunks_in_qdrant(chunks)
    print("\nDocument ingestion complete!")

if __name__ == "__main__":
    ingest_documents()
