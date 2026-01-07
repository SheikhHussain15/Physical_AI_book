# Research: RAG Data Ingestion & Embedding Pipeline

**Date**: 2026-01-07
**Status**: Completed

This document outlines the technology choices for the RAG data ingestion and embedding pipeline.

## Technology Stack

### 1. Web Crawling and Parsing

-   **Decision**: `requests` and `BeautifulSoup4`
-   **Rationale**: These are the standard, lightweight, and most widely used Python libraries for fetching and parsing HTML content. They are simple to use and well-documented, making them ideal for this project's scope.
-   **Alternatives considered**:
    -   **Scrapy**: A more powerful and feature-rich crawling framework. Rejected as it is overly complex for the current requirement of crawling a single, well-structured Docusaurus site.

### 2. Text Chunking

- **Decision**: Custom lightweight chunking logic (character- or token-based)
- **Rationale**: The ingestion pipeline requires only deterministic text splitting for embedding generation. A custom implementation avoids introducing agent-oriented or orchestration frameworks, keeps dependencies minimal, and provides full control over chunk size and overlap. This aligns with the project’s architectural constraint that ingestion remains agent-free.
- **Alternatives considered**:
    - **LangChain text splitters**: Rejected to avoid introducing agent-adjacent abstractions and unnecessary dependency weight.
    - **Sentence-tokenization libraries**: Rejected as overly complex for static documentation content.


### 3. Embedding Model

-   **Decision**: Cohere Embeddings
-   **Rationale**: This was a constraint specified in the feature description. Cohere provides high-quality embedding models suitable for semantic search.
-   **Alternatives considered**:
    -   **OpenAI Embeddings**: Another popular choice, but Cohere was specified.

### 4. Vector Database

-   **Decision**: Qdrant Cloud (Free Tier)
-   **Rationale**: This was a constraint specified in the feature description. Qdrant is a modern, high-performance vector database with a generous free tier, making it suitable for this project.
-   **Alternatives considered**:
    -   **ChromaDB / Pinecone**: Other popular vector databases. Qdrant was specified.

### 5. Dependency Management

-   **Decision**: `uv`
-   **Rationale**: `uv` is a modern, extremely fast Python package installer and resolver. It's a single binary that can replace `pip` and `venv`, simplifying the development workflow and ensuring reproducible environments.
-   **Alternatives considered**:
    -   **pip + venv**: The standard Python tooling. `uv` is a simpler and faster alternative.
    -   **Poetry / PDM**: More comprehensive project management tools. Rejected as slightly too heavyweight for a single-script project. `uv` hits the sweet spot of simplicity and performance.

