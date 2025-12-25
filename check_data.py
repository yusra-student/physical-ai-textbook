import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv(dotenv_path='backend/.env')

qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

try:
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    collection_name = "rag_embedding"
    
    # Check if collection exists
    collections = client.get_collections()
    exists = any(c.name == collection_name for c in collections.collections)
    
    if not exists:
        print(f"Collection '{collection_name}' DOES NOT EXIST.")
    else:
        count = client.count(collection_name=collection_name)
        print(f"Collection '{collection_name}' exists.")
        print(f"Total documents embedded: {count.count}")
        
except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
