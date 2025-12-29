# Actionable Tasks: RAG Backend Refactor – OpenAI Agents SDK

**Completion Status**: 0/17 tasks completed

This document outlines the actionable tasks required to implement the "RAG Backend Refactor – OpenAI Agents SDK" feature. The tasks are organized by phase and user story to enable parallel work and independent testing.

## Phase 1: Setup

- [ ] T001 Set up the development environment by installing dependencies from `backend/pyproject.toml`.
- [ ] T002 Create a `.env` file in the `backend` directory with `OPENAI_API_KEY`, `QDRANT_URL`, and `QDRANT_API_KEY`.

## Phase 2: Foundational Tasks

- [ ] T003 [P] In `backend/agent_service.py`, define the Pydantic models for the chat request (`ChatRequest`) and response (`ChatResponse`) based on `data-model.md`.
- [ ] T004 [P] In a new file `backend/qdrant_tool.py`, create a function to wrap the Qdrant client's search functionality. This function will be used as a tool by the agent.
- [ ] T005 [P] In `backend/agent_service.py`, initialize the OpenAI client and the Qdrant client as singletons.

## Phase 3: User Story 1 - Refactor Agent

- [ ] T006 [US1] In `backend/agent_service.py`, define a new `agents.Agent` with instructions to answer questions based on the provided context.
- [ ] T007 [US1] In `backend/agent_service.py`, register the Qdrant search function from `backend/qdrant_tool.py` as a tool for the agent.
- [ ] T008 [US1] In `backend/agent_service.py`, create an `agents.Runner` for the new agent.
- [ ] T009 [US1] In `backend/agent_service.py`, remove the old chatbot logic.

## Phase 4: User Story 2 - Implement Chat Endpoint

- [ ] T010 [US2] In `backend/main.py`, modify the `/chat` endpoint to call the new agent runner.
- [ ] T011 [US2] In `backend/main.py`, ensure the `/chat` endpoint passes the user's question to the agent.
- [ ] T012 [US2] In `backend/main.py`, ensure the `/chat` endpoint retrieves the grounded answer and source URLs from the agent's response and returns them to the user.

## Phase 5: User Story 3 - Validation and Testing

- [ ] T013 [US3] In `backend/tests/test_agent_service.py`, update the unit tests to reflect the new agent-based implementation.
- [ ] T014 [US3] In `backend/tests/test_main.py`, update the integration tests for the `/chat` endpoint to verify the correct responses.
- [ ] T015 [US3] In `backend/tests/test_retrieval_validation.py`, ensure that the tests for the existing retrieval logic still pass.

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T016 Update `README.md` in the `backend` directory with any changes to the setup or execution instructions.
- [ ] T017 Perform a final code review and cleanup.

## Dependency Graph

- **User Story 1 (US1)**: Depends on Phase 1 and 2.
- **User Story 2 (US2)**: Depends on US1.
- **User Story 3 (US3)**: Depends on US2.

The user stories must be completed in order: US1 -> US2 -> US3.

## Parallel Execution

- Within Phase 2, tasks T003, T004, and T005 can be worked on in parallel.
- Once the foundational tasks are complete, different developers could potentially work on the agent implementation (US1) and test updates (US3) in parallel, but it is recommended to follow the dependency graph.

## Implementation Strategy

The implementation will follow a phased approach, starting with the foundational setup, then implementing the core agent logic, connecting it to the API, and finally, ensuring everything is tested. This aligns with an MVP-first strategy, where the goal is to get a functional, testable slice of the new implementation working as quickly as possible. The MVP for this feature would be the completion of User Story 1 and 2.
