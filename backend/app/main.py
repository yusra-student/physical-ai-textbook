from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import ingest, query, query_selection, conversations, ask
from app.config import settings
import uvicorn

app = FastAPI(title="RAG Chatbot for Published Books API")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(ingest.router, tags=["Ingestion"])
app.include_router(query.router, tags=["Query"])
app.include_router(query_selection.router, tags=["Query"])
app.include_router(conversations.router, tags=["Conversation"])
app.include_router(ask.router, tags=["Ask"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot for Published Books API"}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)