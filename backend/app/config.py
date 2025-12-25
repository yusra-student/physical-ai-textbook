from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Cohere API
    COHERE_API_KEY: str
    COHERE_EMBED_MODEL: str = "embed-english-v3.0"
    COHERE_LLM_MODEL: str = "command-r"
    COHERE_RERANK_MODEL: str = "rerank-english-v3.0"

    # Database
    DATABASE_URL: str
    NEON_DATABASE_URL: Optional[str] = None

    # Qdrant Vector Database
    QDRANT_URL: str
    QDRANT_API_KEY: str

    # Security
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # Application
    ENVIRONMENT: str = "development"
    DEBUG: bool = False
    PORT: int = 8000
    WORKERS: int = 1

    # Rate limiting
    RATE_LIMIT_SECRET: str

    # File upload
    MAX_FILE_SIZE: int = 500 * 1024 * 1024  # 500MB in bytes
    ALLOWED_FILE_TYPES: list = ["application/pdf", "application/epub+zip"]

    class Config:
        env_file = ".env"


settings = Settings()