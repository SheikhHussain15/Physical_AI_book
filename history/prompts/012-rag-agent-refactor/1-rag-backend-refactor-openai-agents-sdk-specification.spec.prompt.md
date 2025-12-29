---
id: 1
title: RAG Backend Refactor - OpenAI Agents SDK Specification
stage: spec
date: 2025-12-28
surface: agent
model: null
feature: 012-rag-agent-refactor
branch: 012-rag-agent-refactor
user: null
command: /sp.specify "RAG Backend Refactor – OpenAI Agents SDK Target audience: - Backend developers refactoring the RAG chatbot agent Objective: - Replace existing chatbot logic with a simple OpenAI Agents SDK–based agent - Preserve existing Qdrant retrieval and FastAPI interface Success criteria: - Agent is implemented using `agents.Agent` and `Runner` - `/chat` endpoint accepts a user question and returns a grounded answer - Responses include source URLs - No non-Agents SDK logic remains Constraints: - Agent framework: OpenAI Agents SDK ONLY - Backend: FastAPI - Language: Python - Vector DB: Qdrant - No frontend or embedding changes"
labels: ["rag", "backend", "refactor", "openai", "agent", "sdk", "specification"]
links:
  spec: specs/012-rag-agent-refactor/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-rag-agent-refactor/spec.md
  - specs/012-rag-agent-refactor/checklists/requirements.md
tests:
  - null
---

## Prompt

RAG Backend Refactor – OpenAI Agents SDK Target audience: - Backend developers refactoring the RAG chatbot agent Objective: - Replace existing chatbot logic with a simple OpenAI Agents SDK–based agent - Preserve existing Qdrant retrieval and FastAPI interface Success criteria: - Agent is implemented using `agents.Agent` and `Runner` - `/chat` endpoint accepts a user question and returns a grounded answer - Responses include source URLs - No non-Agents SDK logic remains Constraints: - Agent framework: OpenAI Agents SDK ONLY - Backend: FastAPI - Language: Python - Vector DB: Qdrant - No frontend or embedding changes

## Response snapshot

- Generated short name: `rag-agent-refactor`
- Determined next feature number: `012`
- Created new branch: `012-rag-agent-refactor`
- Created feature directory: `specs/012-rag-agent-refactor`
- Created prompt history directory: `history/prompts/012-rag-agent-refactor`
- Copied `spec-template.md` to `specs/012-rag-agent-refactor/spec.md`
- Populated `specs/012-rag-agent-refactor/spec.md` with detailed specification.
- Created and validated checklist at `specs/012-rag-agent-refactor/checklists/requirements.md`. All checks passed.

## Outcome

- ✅ Impact: The specification for the "RAG Backend Refactor – OpenAI Agents SDK" feature has been successfully created, detailing user scenarios, functional requirements, and success criteria.
- 🧪 Tests: No tests were created in this phase.
- 📁 Files: Two new files were created: `specs/012-rag-agent-refactor/spec.md` and `specs/012-rag-agent-refactor/checklists/requirements.md`.
- 🔁 Next prompts: The next step is to proceed with planning using `/sp.plan`.
- 🧠 Reflection: The manual creation of the branch and files was necessary due to issues with the `create-new-feature.ps1` script's argument parsing, which needs to be investigated further.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-new-feature.ps1` script failed to execute correctly due to PowerShell argument parsing issues when a long feature description was provided. This necessitated manual branch and directory creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Investigate and fix the `create-new-feature.ps1` script for robust argument parsing, especially with long strings.