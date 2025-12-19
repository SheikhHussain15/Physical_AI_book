<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Modified principles: Spec-First Development -> Spec-first workflow using Spec-Kit Plus, Technical Accuracy and Verifiability -> Technical accuracy from official sources, Reproducibility and Open Deployment -> Reproducible setup and deployment, Clear, Developer-Focused Pedagogy -> Clear, developer-focused writing
- Added sections: Key Standards, Success Criteria
- Removed sections: Objectives, Architecture, Standards
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: none
-->
# AI-Spec–Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first workflow using Spec-Kit Plus
Every feature starts with a clear specification, following the Spec-Kit Plus methodology.

### Technical accuracy from official sources
All technical content must be accurate and verifiable through official documentation or examples.

### Reproducible setup and deployment
The project must be fully reproducible from its repository, ensuring consistent setup and automated deployment.

### Clear, developer-focused writing
The primary audience is developers; explanations should be clear, concise, and focused on practical application.

## Key Standards

- Book to be written with Docusaurus and deployed to GitHub Pages.
- RAG chatbot responses must be grounded solely in the book content or user-selected text.
- Core technology stack includes: OpenAI Agents/ChatKit, FastAPI, Neon serverless Postgres, and Qdrant Cloud.
- All code examples must be runnable, minimal, well-documented, and verifiable.

## Constraints

- All source control operations must be GitHub-based.
- Chatbot responses must not contain hallucinated information.
- The entire project setup and output must be end-to-end reproducible.
- Markdown/MDX only for content.
- Free-tier services only.
- Public, reproducible repository.

## Success Criteria

- A live technical book successfully deployed on GitHub Pages.
- A fully functional embedded RAG chatbot that meets all grounding requirements.
- All project specifications and feature implementations are guided and documented via Spec-Kit Plus.

## Governance

The constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All pull requests and reviews must verify compliance with the constitution. Complexity must be justified.

**Version**: 1.1.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15
