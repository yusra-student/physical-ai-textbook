import asyncio
import logging
from typing import Dict, Any, List
from src.ai_book.tasks.integration.rag_client import RAGClient
from src.ai_book.tasks.engine.scheduler import TaskScheduler
from src.ai_book.tasks.engine.composer import TaskComposer
from src.ai_book.tasks.schema.validator import TaskDefinitionValidator
from src.ai_book.tasks.engine.state import TaskStatus
from src.ai_book.tasks.library.ai_tasks import ask_rag_step_executor # Import the specific executor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

async def run_rag_query_example():
    """
    Demonstrates a direct RAG query using the RAGClient.
    """
    logger.info("--- Running RAG Query Example (Direct RAGClient) ---")
    rag_client = RAGClient()
    question = "What are the main components of the RAG system in the AI book?"
    answer, citations, contexts = await rag_client.query_rag(question)

    logger.info(f"\nQuestion: {question}")
    logger.info(f"Answer: {answer}")
    logger.info(f"Citations: {citations}")
    logger.info(f"Contexts Used (first 2):")
    for ctx in contexts[:2]:
        logger.info(f"  - Source: {ctx.get('source')} | Content: {ctx.get('content', '')[:100]}...")
    logger.info("--- RAG Query Example Finished ---")

async def run_ask_rag_task_example():
    """
    Demonstrates executing a task that includes an 'ask_rag' step type
    via the TaskComposer.
    """
    logger.info("\n--- Running AskRAG Task Example (via TaskComposer) ---")
    scheduler = TaskScheduler()
    validator = TaskDefinitionValidator()
    composer = TaskComposer(scheduler=scheduler, validator=validator)
    
    # Register the 'ask_rag' step executor
    composer.register_step_executor("ask_rag", ask_rag_step_executor)

    # For this example, we also need a 'command' executor for the second step.
    # In a real application, all step executors would be registered at app startup.
    async def dummy_command_executor(step_def: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        cmd_template_str = step_def["parameters"]["command"]
        from jinja2 import Template
        rendered_cmd = Template(cmd_template_str).render(context)
        logger.info(f"DUMMY COMMAND EXECUTOR: Simulating command execution: '{rendered_cmd}'")
        return {"stdout": rendered_cmd, "status": "executed"}

    composer.register_step_executor("command", dummy_command_executor)

    # Define a simple task that uses an 'ask_rag' step
    task_definition = {
        "id": "query_ai_book_rag",
        "name": "Query AI Book RAG System",
        "description": "Task to ask a question to the RAG system and log its answer.",
        "parameters": {
            "query_text": {
                "type": "string",
                "description": "The question to ask the RAG system."
            }
        },
        "steps": [
            {
                "id": "step1_ask_rag",
                "type": "ask_rag",
                "name": "Ask RAG",
                "parameters": {
                    "question": "{{ parameters.query_text }}",
                    "limit": 3
                },
                "output_to": "rag_result", # Store RAG output in context under 'rag_result'
                "on_success": "step2_log_result",
                "on_failure": "END" # Fail task if RAG step fails
            },
            {
                "id": "step2_log_result",
                "type": "command", 
                "name": "Log RAG Result",
                "parameters": {
                    "command": "echo 'RAG Answer: {{ rag_result.answer }} \nCitations: {{ rag_result.citations }}'"
                },
                "on_success": "END"
            }
        ]    
    }
    
    # Validate task definition
    try:
        validator.validate_task_definition(task_definition)
        logger.info("Task definition validated successfully.")
    except Exception as e:
        logger.error(f"Task definition validation failed: {e}")
        return

    # Execute the task
    initial_context = {"query_text": "What is the role of Qdrant in our system architecture?"}
    final_state = await composer.execute_task(task_definition, initial_context)

    logger.info(f"\nTask '{task_definition['id']}' finished with status: {final_state.status}")
    if final_state.status == TaskStatus.COMPLETED and final_state.result:
        rag_output = final_state.result.get('rag_result', {})
        logger.info(f"Final Task Result (RAG Answer): {rag_output.get('answer')}")
        logger.info(f"Final Task Result (RAG Citations): {rag_output.get('citations')}")
    else:
        logger.error(f"Task failed or did not complete successfully. Error: {final_state.error}")
    logger.info("--- AskRAG Task Example Finished ---")

if __name__ == "__main__":
    # Ensure GOOGLE_API_KEY is set in your environment or .env file
    # For local execution, you might need to run a Qdrant and Redis instance
    # and have the `sentence-transformers` library installed for embeddings/reranking.
    # (pip install "qdrant-client[fastembed]" "sentence-transformers" "redis" "rq" "fastapi" "uvicorn" "pydantic" "pydantic-settings" "jsonschema" "Jinja2" "python-dotenv" "passlib[bcrypt]")
    
    # Run the examples
    asyncio.run(run_rag_query_example())
    asyncio.run(run_ask_rag_task_example())
