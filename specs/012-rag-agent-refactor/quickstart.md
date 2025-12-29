# Quickstart: RAG Backend Refactor

This document provides instructions on how to run and test the refactored RAG backend.

## Prerequisites

- Python 3.11+
- Pip
- Access to an OpenAI API key
- Access to a Qdrant instance

## Setup

1.  **Install dependencies**:
    ```bash
    cd backend
    pip install .
    ```

2.  **Configure environment variables**:
    Create a `.env` file in the `backend` directory with the following content:
    ```
    OPENAI_API_KEY="your_openai_api_key"
    QDRANT_URL="your_qdrant_url"
    QDRANT_API_KEY="your_qdrant_api_key"
    ```

## Running the service

1.  **Start the FastAPI server**:
    ```bash
    cd backend
    uvicorn main:app --reload
    ```
    The server will be available at `http://127.0.0.1:8000`.

## Testing the endpoint

You can use `curl` or any API client to test the `/chat` endpoint.

```bash
curl -X POST "http://127.0.0.1:8000/chat" \
-H "Content-Type: application/json" \
-d '{"question": "What is the capital of France?"}'
```

The expected response should be a JSON object containing the answer and sources:
```json
{
  "answer": "Paris is the capital of France.",
  "sources": ["http://example.com/source1", "http://example.com/source2"]
}
```
