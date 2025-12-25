import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from app.config import settings
from sqlalchemy import create_engine
from sqlalchemy.exc import OperationalError
from sqlalchemy_utils import database_exists, create_database
from app.models.database import Base
import uvicorn
from app.main import app

def create_db_if_not_exists():
    """Create database if it doesn't exist"""
    try:
        # Create engine
        engine = create_engine(settings.DATABASE_URL)
        
        # Check if database exists, if not create it
        if not database_exists(engine.url):
            create_database(engine.url)
            print("Database created successfully!")
        else:
            print("Database already exists.")
        
        # Try to create tables
        Base.metadata.create_all(bind=engine)
        print("Tables created successfully!")
        
    except OperationalError as e:
        print(f"Could not connect to database: {e}")
        print("Make sure PostgreSQL is running.")
        return False
    except Exception as e:
        print(f"Error creating database: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("Starting backend server...")
    print(f"Database URL: {settings.DATABASE_URL}")
    
    # Try to create database if it doesn't exist
    db_ok = create_db_if_not_exists()
    
    if db_ok:
        print("Starting server with database connection...")
        uvicorn.run("app.main:app", host="0.0.0.0", port=settings.PORT, reload=True)
    else:
        print("Starting server without database connection...")
        # Start the server anyway, with error handling for database operations
        uvicorn.run("app.main:app", host="0.0.0.0", port=settings.PORT, reload=True)