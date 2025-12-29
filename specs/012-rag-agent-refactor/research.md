# Research: RAG Backend Refactor – OpenAI Agents SDK

This document summarizes the research conducted to resolve ambiguities in the technical plan and to establish best practices for the implementation.

## 1. Performance Goals

### Decision
- **Response Time**: The target for the `/chat` endpoint is a p95 response time of **under 3 seconds**.
- **Memory/CPU**: The agent service should operate within **< 1 GB RAM** and **1 vCPU** to align with common free-tier cloud service limits.
- **Request Volume**: The existing request volume is assumed to be handled, and the refactor should not introduce performance regressions.

### Rationale
- The 1-3 second response time is the industry standard for interactive RAG chatbots to ensure a smooth user experience.
- The memory and CPU constraints are based on typical free-tier limits of cloud providers, ensuring the application is cost-effective and easily deployable.
- Since this is a refactor of an existing component, maintaining the current performance is the primary goal.

### Alternatives Considered
- Tighter response time goals (e.g., <1s) were considered but deemed overly ambitious for an initial refactor without a clear performance bottleneck to solve.

## 2. Integration Best Practices: FastAPI, Qdrant, and OpenAI Agents SDK

### Decision
The following best practices will be adopted for the integration:

#### FastAPI
-   **Asynchronous Endpoints**: All I/O operations (Qdrant, OpenAI API calls) will be fully asynchronous using `async/await`.
-   **Pydantic Models**: Request and response models will be strictly defined using Pydantic for validation and documentation.
-   **Singleton Client**: The Qdrant client will be initialized once and reused across the application's lifespan.

#### Qdrant
-   **Tool Integration**: Qdrant search functionality will be wrapped as a "tool" that the OpenAI agent can invoke.
-   **Metadata Filtering**: Retrieved documents will be filtered using metadata to ensure relevance.

#### OpenAI Agents SDK
-   **Agent Definition**: A dedicated `agents.Agent` will be defined with clear instructions.
-   **Tool-Based Retrieval**: The agent will use the Qdrant search tool to retrieve context before generating a response.

### Rationale
These practices ensure a scalable, maintainable, and efficient implementation. Using `async` in FastAPI is crucial for I/O-bound applications. The singleton pattern for clients prevents resource exhaustion. Integrating retrieval as a tool for the agent is the standard pattern for building RAG capabilities with the OpenAI Agents SDK.

### Alternatives Considered
- A simpler, non-agent-based approach was implicitly rejected by the feature's core requirement to use the OpenAI Agents SDK.
- Direct integration of Qdrant logic inside the agent's main execution flow was considered, but using the "tool" abstraction provides better separation of concerns.
