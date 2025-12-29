# RAG Backend Refactor – OpenAI Agents SDK

This directory contains Python scripts for the refactored RAG (Retrieval-Augmented Generation) agent service.

## Setup

1.  **Install Dependencies**: This project uses `pip` for package management. To install the required packages, navigate to this `backend` directory and run:

    ```bash
    pip install cohere qdrant-client beautifulsoup4 requests langchain langchain-text-splitters python-dotenv pytest fastapi openai uvicorn agents
    ```

    Note: The `agents` library might need to be installed separately or from a specific source if not available on PyPI.

2.  **Environment Variables**: Create a `.env` file in this directory. It must contain the following variables:

    ```
    OPENAI_API_KEY="your_openai_api_key_here"
    QDRANT_URL="your_qdrant_cloud_url_here"
    QDRANT_API_KEY="your_qdrant_api_key_here"
    ```

    Replace the placeholder values with your actual credentials from OpenAI and [Qdrant Cloud](https://qdrant.tech/cloud/).

## How to Run the RAG Agent Service (`main.py`)

This script runs a FastAPI application that exposes an OpenAI Agents SDK-based RAG chatbot.

1.  **Start the FastAPI server**:

    ```bash
    uvicorn main:app --reload
    ```

    The `--reload` flag is optional and useful for development, as it automatically reloads the server on code changes.

2.  **Making Queries to the Agent**:
    Once the server is running, you can interact with the agent via its API endpoint. The main endpoint is `/chat`.

    ### Using `curl`

    ```bash
    curl -X POST "http://127.0.0.1:8000/chat" \
         -H "Content-Type: application/json" \
         -d '{"question": "What are the core components of ROS2?"}'
    ```

    ### Using a Python Script

    ```python
    import requests
    import json

    url = "http://127.0.0.1:8000/chat"
    headers = {"Content-Type": "application/json"}
    payload = {"question": "Tell me about the main features of Isaac Sim."}

    response = requests.post(url, headers=headers, data=json.dumps(payload))
    print(json.dumps(response.json(), indent=2))
    ```

    ## Accessing API Documentation

    While the server is running, you can access the interactive OpenAPI (Swagger UI) documentation at:

    [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)

## Ingestion Pipeline and Retrieval Validation

The ingestion pipeline and retrieval validation functionalities are no longer part of this backend service. Their implementations (previously in `main.py` and `retrieval_validation.py`) have been removed or refactored out.

## How to Run Tests

The project includes unit and integration tests for key functionality. To run the tests, use `pytest` from the `backend` directory:

```bash
pytest
```