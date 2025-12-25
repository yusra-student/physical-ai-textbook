from fastapi import APIRouter, HTTPException, Depends
from app.models.database import get_db
from app.models.pydantic_schemas import (
    CreateConversationRequest, CreateConversationResponse,
    ConversationResponse, Message
)
from app.models.document import Document
from app.models.conversation import Conversation
from app.services.db_service import db_service
from sqlalchemy.orm import Session
import uuid
from datetime import datetime

router = APIRouter()


@router.post("/conversations", status_code=201)
async def create_conversation(
    request: CreateConversationRequest,
    db: Session = Depends(get_db)
):
    """
    Initialize a new conversation session with a book
    """
    try:
        # 1. Verify document exists
        document = db.query(Document).filter(Document.id == request.document_id).first()
        if not document:
            raise HTTPException(status_code=404, detail={
                "error": "Document not found",
                "message": f"Document with ID {request.document_id} not found",
                "code": "DOCUMENT_NOT_FOUND"
            })

        # 2. Generate a session ID
        session_id = str(uuid.uuid4())
        current_time = datetime.utcnow().isoformat() + "Z"

        # 3. Store session metadata
        # We could store session metadata in a separate table if needed
        # For now, we'll just return the session information
        # The actual conversation entries will be created when queries are made

        return CreateConversationResponse(
            session_id=session_id,
            document_id=request.document_id,
            created_at=current_time,
            ttl_hours=24
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error during conversation creation",
            "message": str(e),
            "code": "SERVER_ERROR"
        })


@router.get("/conversations/{session_id}")
async def get_conversation(
    session_id: str,
    db: Session = Depends(get_db)
):
    """
    Get the history of messages in a conversation session
    """
    try:
        # 1. Validate session exists by checking if there are any conversations with this session_id
        conversation_entries = db.query(Conversation).filter(
            Conversation.session_id == session_id
        ).all()

        if not conversation_entries:
            raise HTTPException(status_code=404, detail={
                "error": "Conversation session not found",
                "message": f"Conversation session with ID {session_id} not found",
                "code": "CONVERSATION_NOT_FOUND"
            })

        # 2. Format conversation entries as Message objects
        messages = []
        for conv_entry in conversation_entries:
            # Create a user message
            user_message = Message(
                id=f"query_{conv_entry.id}",
                type="user",
                content=conv_entry.query,
                timestamp=conv_entry.created_at.isoformat() + "Z"
            )
            messages.append(user_message)

            # Create an assistant message
            assistant_message = Message(
                id=f"answer_{conv_entry.id}",
                type="assistant",
                content=conv_entry.answer,
                timestamp=conv_entry.created_at.isoformat() + "Z"  # Same timestamp as entry
            )
            messages.append(assistant_message)

        # 3. Return conversation history
        return ConversationResponse(
            session_id=session_id,
            document_id=str(conversation_entries[0].document_id),  # All entries in session have same document
            message_count=len(messages),
            messages=messages
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail={
            "error": "Server error retrieving conversation",
            "message": str(e),
            "code": "SERVER_ERROR"
        })