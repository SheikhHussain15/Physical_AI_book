# Implementation Plan: RAG Data Ingestion & Embedding Pipeline

**Branch**: `013-rag-ingestion-embedding-pipeline` | **Date**: 2026-01-07 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/013-rag-ingestion-embedding-pipeline/spec.md`

## Summary

This plan outlines the implementation of a data ingestion and embedding pipeline for a technical book website. The pipeline will crawl the deployed Docusaurus book from a given URL, extract content, generate embeddings using Cohere, and store them in a Qdrant vector database. The entire process will be orchestrated by a Python script, managed with `uv`.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: `uv`, `requests`, `beautifulsoup4`, `cohere`, `qdrant-client`
**Storage**: Qdrant Cloud (Free Tier)
**Testing**: pytest
**Target Platform**: Any platform with Python 3.11+
**Project Type**: Backend script
**Performance Goals**: Ingestion of a 500-page book in under 30 minutes.
**Constraints**: Must operate within the free tiers of Cohere and Qdrant Cloud.
**Agent Usage Constraint**:
- No agent framework is used in this spec.
- OpenAI Agents SDK is reserved exclusively for Spec-3 (retrieval + reasoning).
- This pipeline produces data only.
**Scale/Scope**: Designed to handle a single book of up to 500 pages.
**Text Chunking**: Custom token / character-based chunking (no agent or orchestration frameworks)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [X]  **Spec-first workflow using Spec-Kit Plus**: This plan is derived from a detailed specification.
- [X]  **Technical accuracy from official sources**: All library usage will be based on official documentation.
- [X]  **Reproducible setup and deployment**: `uv` will be used for reproducible dependency management.
- [X]  **Clear, developer-focused writing**: The code will be structured and commented for clarity.

## Project Structure

### Documentation (this feature)

```text
specs/013-rag-ingestion-embedding-pipeline/
├── plan.md              # This file
├── research.md          # Technology choices and rationale
├── data-model.md        # Data model for content chunks
├── quickstart.md        # Instructions to run the pipeline
└── tasks.md             # To be created by /sp.tasks
```

### Source Code (repository root)

```text
backend/
└── src/
    └── main.py
```

**Structure Decision**: A single `backend` directory is created to house the ingestion script. This keeps the implementation isolated and simple, aligning with its purpose as a backend-only tool.

## Implementation Phases

### Phase 1: Project Setup

1.  Create the `backend/` directory.
2.  Initialize a new Python virtual environment using `uv`.
3.  Install necessary dependencies: `requests`, `beautifulsoup4`, `cohere`,`qdrant-client`, `python-dotenv`.
4.  Create the main entry point file: `backend/src/main.py`.
5.  Set up environment variable handling for API keys (`.env` file).

### Phase 2: Content Extraction and Chunking

1.  Implement a function to fetch the HTML content of a given URL.
2.  Implement a web crawler to discover all unique page URLs starting from a root URL.
3.  Use `BeautifulSoup4` to parse the HTML and extract the main textual content from the pages.
4.  Implement a custom chunking function based on character or token limits with overlap
to break down the extracted text into smaller, semantically meaningful chunks.

### Phase 3: Embedding Generation

1.  Integrate the Cohere API client.
2.  Implement a function that takes a list of text chunks and generates embeddings for them using a specified Cohere model.

### Phase 4: Vector Storage

1.  Integrate the Qdrant API client.
2.  Configure a new Qdrant collection with appropriate vector parameters.
3.  Implement a function to store the text chunks and their corresponding embeddings, along with metadata (source URL), into the Qdrant collection.

### Phase 5: Pipeline Orchestration

1.  Create a `main()` function in `main.py` that orchestrates the entire pipeline:
    -   Crawls the site.
    -   Extracts and chunks content.
    -   Generates embeddings.
    -   Stores data in Qdrant.
2.  Add logging to provide visibility into the pipeline's progress and any errors.

## Complexity Tracking

No violations of the constitution that require justification.