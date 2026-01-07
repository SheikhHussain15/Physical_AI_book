# Quickstart: RAG Ingestion Pipeline

**Date**: 2026-01-07
**Status**: Draft

This guide provides instructions to set up and run the data ingestion and embedding pipeline.

## Prerequisites

-   Python 3.11+
-   `uv` installed (`pip install uv`)
-   API keys for Cohere and Qdrant Cloud

## Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd <repository_name>
    ```

2.  **Create the virtual environment and install dependencies**:
    Navigate to the `backend` directory and run `uv`:
    ```bash
    cd backend
    uv venv
    uv pip install -r requirements.txt
    ```
    *(Note: `requirements.txt` will be generated during implementation)*

3.  **Configure environment variables**:
    Create a `.env` file in the `backend` directory and add your API keys:
    ```
    COHERE_API_KEY="your_cohere_api_key"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_URL="your_qdrant_cloud_url"
    ```

## Running the Pipeline

1.  **Activate the virtual environment**:
    ```bash
    source .venv/bin/activate  # on Linux/macOS
    .venv\Scripts\activate    # on Windows
    ```

2.  **Run the ingestion script**:
    Execute the `main.py` script from within the `backend` directory:
    ```bash
    python src/main.py --start-url <your_docusaurus_book_url>
    ```

The script will crawl the website, process the content, and populate your Qdrant collection with the embeddings.
