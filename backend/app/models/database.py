from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from app.config import settings

# Database connection setup
def get_engine():
    return create_engine(settings.DATABASE_URL)

engine = None  # Will be initialized when needed
SessionLocal = None  # Will be initialized when needed

Base = declarative_base()

def init_db():
    """Initialize the database engine and session"""
    global engine, SessionLocal
    engine = get_engine()
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Dependency to get DB session
def get_db():
    if SessionLocal is None:
        init_db()
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()