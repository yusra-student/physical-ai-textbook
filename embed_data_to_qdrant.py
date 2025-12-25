#!/usr/bin/env python3
"""
Script to embed data into Qdrant vector database using Cohere embeddings
"""

import asyncio
import os
import uuid
from typing import List, Dict
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import the services from the backend
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from app.services.cohere_service import CohereService
from app.services.qdrant_service import qdrant_service
from app.utils.text_processing import text_processing_service


async def read_text_file(file_path: str) -> str:
    """
    Read text content from a file
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    return content


async def embed_and_store_data(file_path: str, document_id: str, document_title: str):
    """
    Main function to read data, generate embeddings, and store in Qdrant
    """
    print(f"Starting to embed data from {file_path}")
    
    # Step 1: Read the text data
    print("Step 1: Reading text data...")
    text_content = await read_text_file(file_path)
    print(f"Text length: {len(text_content)} characters")
    
    # Step 2: Process the text into chunks
    print("Step 2: Processing text into chunks...")
    chunks = await text_processing_service.chunk_text(text_content)
    print(f"Number of chunks created: {len(chunks)}")
    
    # Step 3: Generate embeddings for each chunk
    print("Step 3: Generating embeddings...")
    cohere_service = CohereService()
    
    # Extract text from chunks for embedding
    chunk_texts = [chunk["text"] for chunk in chunks]
    
    # Generate embeddings using Cohere
    embeddings = cohere_service.embed_documents(chunk_texts)
    print(f"Generated {len(embeddings)} embeddings")
    
    # Verify embedding dimensions
    if embeddings and len(embeddings[0]) != 1024:  # Cohere embed-english-v3.0 has 1024 dimensions
        print(f"Warning: Expected embeddings of dimension 1024, got {len(embeddings[0])}")
    else:
        print(f"Embedding dimension: {len(embeddings[0])}")
    
    # Step 4: Store embeddings in Qdrant
    print("Step 4: Storing embeddings in Qdrant...")
    
    # Create the collection if it doesn't exist
    await qdrant_service.create_collection_if_not_exists()
    
    # Index the chunks with embeddings in Qdrant
    await qdrant_service.index_chunks(
        chunks=chunks,
        embeddings=embeddings,
        document_id=document_id,
        document_title=document_title
    )
    
    print(f"Successfully stored {len(chunks)} chunks in Qdrant collection")


async def verify_embeddings_stored(document_title: str):
    """
    Verify that embeddings were stored correctly
    """
    print("\nStep 5: Verifying stored embeddings...")
    
    # Create a simple test query to verify if the data is retrievable
    test_query = "What is artificial intelligence?"
    cohere_service = CohereService()
    query_embedding = cohere_service.embed_query(test_query)
    
    # Search for similar content in Qdrant
    results = await qdrant_service.search_similar(
        query_embedding=query_embedding,
        top_k=5
    )
    
    print(f"Found {len(results)} similar chunks for query: '{test_query}'")
    
    if results:
        print("Sample retrieved content:")
        for i, (payload, score) in enumerate(results[:3]):  # Show top 3 results
            print(f"  {i+1}. Score: {score:.3f}")
            print(f"     Content preview: {payload['text'][:100]}...")
            print()
    else:
        print("No results found - embeddings may not have been stored properly")
        
    # Also check collection size
    try:
        collection_info = qdrant_service.client.get_collection(qdrant_service.collection_name)
        print(f"Total vectors in collection '{qdrant_service.collection_name}': {collection_info.points_count}")
    except Exception as e:
        print(f"Could not get collection info: {e}")


async def main():
    """
    Main function to run the embedding process
    """
    print("Qdrant Data Embedding Script")
    print("="*50)
    
    # Configuration
    file_path = "./data_to_embed/sample_ai_content.txt"
    document_id = str(uuid.uuid4())  # Generate a unique ID for this document
    document_title = "Sample AI Content"
    
    # Check if file exists
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} does not exist!")
        return
    
    try:
        # Embed and store the data
        await embed_and_store_data(file_path, document_id, document_title)
        
        # Verify embeddings were stored
        await verify_embeddings_stored(document_title)
        
        print("\nData embedding completed successfully!")
        
    except Exception as e:
        print(f"\nError during embedding process: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())