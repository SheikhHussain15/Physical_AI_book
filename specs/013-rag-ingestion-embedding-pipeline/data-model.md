# Data Model: RAG Ingestion Pipeline

**Date**: 2026-01-07
**Status**: Draft

This document defines the data structures used in the RAG ingestion pipeline.

## Entities

### 1. ContentChunk

Represents a semantically coherent block of text extracted from the source website.

**Fields**:

| Name         | Type   | Description                                           |
|--------------|--------|-------------------------------------------------------|
| `doc_id`     | String | A unique identifier for the chunk (e.g., a hash).     |
| `text`       | String | The raw text content of the chunk.                    |
| `metadata`   | Object | A dictionary containing metadata about the chunk.     |

**Metadata Object**:

| Name         | Type   | Description                                           |
|--------------|--------|-------------------------------------------------------|
| `source_url` | String | The URL of the page from which the chunk was extracted. |
| `title`      | String | The title of the source page.                         |

### 2. VectorEmbedding

Represents the vector embedding of a `ContentChunk`, stored in the Qdrant vector database. In Qdrant, this corresponds to a "point".

**Fields**:

| Name      | Type          | Description                                                    |
|-----------|---------------|----------------------------------------------------------------|
| `id`      | String / UUID | The unique identifier of the point, matching `ContentChunk.doc_id`. |
| `vector`  | Array[Float]  | The dense vector representation of the text.                   |
| `payload` | Object        | The metadata associated with the vector, equivalent to the `ContentChunk` itself. |
