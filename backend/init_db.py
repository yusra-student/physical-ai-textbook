import sys
import os

# Add the project root to the path so we can import our models
sys.path.insert(0, os.path.dirname(__file__))

# Load environment variables before importing other modules
from dotenv import load_dotenv
load_dotenv()

from app.models.database import Base, init_db
from app.models.document import Document
from app.models.chunk import TextChunk
from app.models.conversation import Conversation
from app.models.job import Job
from app.models.user_selection import UserSelection

def create_tables():
    """Create all database tables"""
    print("Initializing database connection...")
    init_db()
    from app.models.database import engine
    print("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully!")

if __name__ == "__main__":
    create_tables()