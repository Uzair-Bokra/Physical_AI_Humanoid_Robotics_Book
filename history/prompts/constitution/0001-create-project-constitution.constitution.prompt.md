---
id: 0001
title: Create Project Constitution
stage: constitution
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "spec-driven-development", "ai-book", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: AI-Native Book Creation with Integrated RAG Chatbot

Purpose:
Create a unified, production-ready AI-native book using Docusaurus, authored via Claude Code and Spec-Kit Plus, and deployed to GitHub Pages. The book must include an embedded Retrieval-Augmented Generation (RAG) chatbot capable of answering questions strictly from the book's content, including user-selected text.

Core principles:
- Spec-driven development (all outputs follow explicit specifications)
- Accuracy and faithfulness to source content
- Modularity and maintainability (book + chatbot as cohesive system)
- AI-native authoring (human-guided, AI-executed)
- Transparency and traceability of generated content

Key standards:
- Documentation framework: Docusaurus
- Content format: Markdown (.md) files only
- Authoring tools: Claude Code + Spec-Kit Plus
- Deployment target: GitHub Pages
- Book structure: Modules → Chapters → Sections
- Clear separation between content, configuration, and infrastructure
- Consistent technical tone (developer / advanced learner audience)

RAG Chatbot standards:
- Architecture: Retrieval-Augmented Generation (RAG)
- Backend: FastAPI
- LLM integration: OpenAI Agents / ChatKit SDKs
- Vector database: Qdrant Cloud (Free Tier)
- Metadata + state storage: Neon Serverless Postgres
- Retrieval scope:
  - Full-book semantic search
  - User-selected text–only answering mode
- Hallucination prevention:
  - Answers must be grounded strictly in retrieved context
  - If answer is not found, respond with explicit uncertainty
- Chatbot must be embeddable directly into the published Docusaurus site

Content integrity rules:
- No fabricated facts or undocumented claims
- Explanations must be technically accurate and reproducible
- Code examples must be valid, minimal, and runnable
- Clearly distinguish between concepts, implementation, and examples

Constraints:
- All book content stored as `.md` files
- No external proprietary dependencies beyond specified tools
- Free-tier compatible infrastructure only
- Chatbot must function independently after deployment
- No hard-coded secrets or API keys in repository

Success criteria:
- Book builds successfully with Docusaurus
- Deployed site is accessible via GitHub Pages
- Embedded RAG chatbot answers questions accurately from book content
- Selected-text question answering works as specified
- System follows Spec-Kit Plus structure and conventions
- Project is reproducible from repository setup instructions

## Response snapshot

Successfully created the project constitution for the AI-Native Book Creation with Integrated RAG Chatbot project, establishing six core principles and governance structure.

## Outcome

- ✅ Impact: Established foundational constitution with 6 core principles
- 🧪 Tests: none
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Plan feature specifications based on constitution
- 🧠 Reflection: Constitution provides clear governance framework for AI-book project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A