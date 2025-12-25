# Testing Runbooks and CI/CD Integration

This document provides guidelines for testing the Intelligent Systems Integration feature, including test execution procedures, environment setup, and integration with Continuous Integration/Continuous Deployment (CI/CD) pipelines.

## 1. Local Testing Runbook

### 1.1. Prerequisites

-   **Python Environment**: Python 3.10+ installed.
-   **Dependencies**: Install project dependencies.
    ```bash
    pip install -r requirements.txt
    # Note: A requirements.txt will need to be generated/maintained.
    # Essential packages include:
    # fastapi, uvicorn, pydantic, pydantic-settings, redis, rq, qdrant-client,
    # sentence-transformers, jsonschema, Jinja2, google-generativeai, python-dotenv,
    # passlib[bcrypt], pytest, pytest-asyncio
    ```
-   **Docker**: Docker Desktop or Docker Engine installed and running.
-   **Service Containers**: Ensure Redis and Qdrant services are running via Docker Compose.
    ```bash
    docker-compose -f docker/docker-compose.yml up -d redis qdrant
    ```
-   **API Key**: A `.env` file in the project root containing `GOOGLE_API_KEY=YOUR_API_KEY`.

### 1.2. Running Unit and Integration Tests

To execute all Python tests:

```bash
pytest
```

To run tests within a specific module (e.g., RAG components):

```bash
pytest tests/rag/
```

To execute specific integration tests:

```bash
pytest tests/integration/
```

### 1.3. Running the FastAPI Application Locally

To test the API endpoints:

1.  **Start the FastAPI Server**:
    ```bash
    uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
    ```
    *(Note: You will need to create a `src/main.py` (or similar entry point) that instantiates the FastAPI app and includes all relevant API routers.)*
2.  **Start an RQ Worker**: In a separate terminal, run the RQ worker to process background tasks.
    ```bash
    # Assuming you have a script for this, e.g., worker.py
    python worker.py
    # Or directly from scheduler if __main__ is implemented
    python -m src.ai_book.tasks.engine.scheduler 
    ```
    *(You will need to ensure the worker script correctly imports `TaskScheduler` and calls its `run_worker()` method.)*
3.  **Interact with API**: Use tools like Postman, Insomnia, or `curl` to send requests to `http://localhost:8000/api/...`.

### 1.4. Manual RAG Ingestion

To manually ingest documentation into Qdrant for local development/testing:

1.  Ensure Qdrant is running (`docker-compose up -d qdrant`).
2.  Execute an ingestion script.
    ```bash
    # Example command, you will need to create this script
    python scripts/ingest_documents.py --docs-path docs/
    ```
    *(This script should utilize the `DocExtractor`, `Chunker`, `Embedder`, and `QdrantLoader` components.)*

## 2. CI/CD Integration

### 2.1. Automated Testing Workflow

A GitHub Actions workflow (e.g., located at `.github/workflows/ci.yml`) should be configured to automatically:

1.  **Trigger**: On `push` events to `main` branch and `pull_request` events targeting `main`.
2.  **Environment Setup**:
    -   Checkout the repository code.
    -   Set up the Python environment (specify version, e.g., 3.10).
    -   Install project dependencies (`pip install -r requirements.txt`).
    -   Start Docker Compose services (Redis, Qdrant) necessary for integration tests.
3.  **Run Tests**:
    -   Execute `pytest` for all discovered unit and integration tests.
    -   Generate test reports (e.g., JUnit XML format) for better integration with GitHub.
4.  **Code Quality Checks**:
    -   Run Linters (e.g., `ruff check .`, `flake8 src/`).
    -   Execute Type Checkers (e.g., `mypy src/`).

### 2.2. Deployment Automation

-   **Staging Environment**: Implement automated deployments to a staging environment upon successful completion of the CI workflow on `main`.
-   **Production Environment**: Require manual approval for deployments to the production environment after successful testing in staging.
-   **Rollback Strategy**: Maintain a clear and tested rollback strategy for all deployments.

### 2.3. Performance Testing in CI/CD

-   **Lightweight Performance Tests**: Integrate basic performance smoke tests (e.g., for critical API endpoints) directly into the CI pipeline.
-   **Full Load Testing**: Trigger more extensive load and performance test jobs to run on a dedicated performance testing environment after successful deployment to staging.

### 2.4. Security Scanning

-   **SAST (Static Application Security Testing)**: Integrate tools like Bandit for Python code and Trivy for Docker images into the CI pipeline to identify potential security vulnerabilities.
-   **Dependency Vulnerability Scanning**: Automate scanning of third-party dependencies for known security flaws.

## 3. Runbooks for Production Operations

-   **Monitoring Dashboards**: Provide links and guidance to relevant Grafana/Prometheus dashboards for monitoring RAG, Auth, and Task Engine metrics and logs.
-   **Alerting**: Document alert thresholds, notification channels, and on-call rotations for critical system issues.
-   **Troubleshooting Guides**:
    -   "RAG queries returning irrelevant results": Steps for diagnosing (e.g., check Qdrant index health, review ingestion logs, inspect LLM prompts).
    -   "Tasks stuck in PENDING or FAILED": Steps for diagnosing (e.g., check RQ worker logs, Redis connectivity, examine task definition).
    -   "Authentication failures": Steps for diagnosing (e.g., check API key validity, user roles and permissions, audit logs).
-   **Backup and Restore**: Document procedures for backing up and restoring Qdrant collections, Redis data, and other persistent data stores.
