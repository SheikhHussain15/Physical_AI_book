# Tasks: RAG Data Ingestion & Embedding Pipeline

**Input**: Design documents from `specs/013-rag-ingestion-embedding-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: Not explicitly requested in the spec, so no test tasks will be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to the repository root.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure.

- [ ] T001 Create the project directory `backend/src`.
- [ ] T002 Initialize `uv` in the `backend/` directory.
- [ ] T003 Create `backend/requirements.txt` with initial dependencies: `requests`, `beautifulsoup4`, `cohere`, `qdrant-client`, `python-dotenv`.
- [ ] T004 Create the main entry file `backend/src/main.py`.
- [ ] T005 Create a placeholder `.env` file in the `backend` directory.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [ ] T006 [P] In `backend/src/main.py`, implement a function to load environment variables from the `.env` file.
- [ ] T007 [P] In `backend/src/main.py`, implement a function to initialize and return the Cohere client.
- [ ] T008 [P] In `backend/src/main.py`, implement a function to initialize and return the Qdrant client.
- [ ] T009 In `backend/src/main.py`, implement a function to create and configure the Qdrant collection if it doesn't exist.

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Content Ingestion Pipeline (Priority: P1) 🎯 MVP

**Goal**: Implement the core data ingestion pipeline that crawls a website, extracts content, generates embeddings, and stores them in Qdrant.

**Independent Test**: Can be fully tested by providing a URL and checking if the Qdrant collection is populated with content chunks and embeddings.

### Implementation for User Story 1

- [ ] T010 [US1] In `backend/src/main.py`, implement a function `fetch_html(url)` to get the HTML content of a page.
- [ ] T011 [US1] In `backend/src/main.py`, implement a crawler function `crawl_site(start_url)` that discovers all unique, same-domain URLs starting from the `start_url`.
- [ ] T012 [US1] In `backend/src/main.py`, implement a function `extract_text_from_html(html)` to parse HTML using BeautifulSoup and extract the main text content.
- [ ] T013 [US1] In `backend/src/main.py`, implement a function `chunk_text(text)` for custom character or token-based text splitting.
- [ ] T014 [US1] In `backend/src/main.py`, implement a function `generate_embeddings(chunks)` that uses the Cohere client to generate embeddings for a list of text chunks.
- [ ] T015 [US1] In `backend/src/main.py`, implement a function `store_in_qdrant(chunks, embeddings)` to save the data to the Qdrant collection.
- [ ] T016 [US1] In `backend/src/main.py`, implement the main orchestration function `run_ingestion_pipeline(start_url)` that calls the functions from T010-T015 in sequence.

**Checkpoint**: User Story 1 should be fully functional.

---

## Phase 4: User Story 2 - Verifying Search Relevance (Priority: P2)

**Goal**: Add a simple search functionality to test the relevance of the ingested content.

**Independent Test**: Can be tested by running the script with a search query and checking the relevance of the output.

### Implementation for User Story 2

- [ ] T017 [US2] In `backend/src/main.py`, implement a function `search(query)` that takes a text query, generates an embedding for it, and performs a search in the Qdrant collection.
- [ ] T018 [US2] In `backend/src/main.py`, enhance the main execution block to handle command-line arguments, allowing the user to either run the ingestion pipeline or perform a search. For example: `python src/main.py --ingest --url <url>` or `python src/main.py --search "my query"`.

**Checkpoint**: User Story 2 is testable and provides a way to validate the pipeline.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect the overall quality of the script.

- [ ] T019 [P] Add comprehensive logging throughout all functions in `backend/src/main.py`.
- [ ] T020 [P] Implement robust error handling (e.g., for network issues, API errors) in `backend/src/main.py`.
- [ ] T021 [P] Refine the command-line interface in `backend/src/main.py` for better usability.
- [ ] T022 Validate the process by running through the steps in `quickstart.md`.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** must be completed before any other phase.
- **Foundational (Phase 2)** must be completed before the user story phases.
- **User Story 1 (Phase 3)** is the MVP and should be completed first.
- **User Story 2 (Phase 4)** depends on User Story 1.
- **Polish (Phase 5)** can be done last.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test the ingestion pipeline.

### Incremental Delivery

1.  Complete Setup, Foundational, and User Story 1 to deliver the core ingestion capability.
2.  Add User Story 2 to provide a validation mechanism.
3.  Complete the Polish phase to improve robustness and usability.
