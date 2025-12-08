# ADR-0003: Documentation and Content Delivery

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-physical-ai-ebook
- **Context:** The Physical AI E-Book needs a documentation platform that supports technical content (code examples, diagrams), deploys to GitHub Pages, and balances simplicity for content authors with interactivity for learners. The platform must handle 5 modules, 35+ chapters, 80+ code examples, and 20+ architecture diagrams.

## Decision

We will use Docusaurus v3 with a hybrid Markdown/MDX approach:

- **Documentation Platform**: Docusaurus v3.9.2
  - React-based static site generator
  - Built-in versioning, search, and i18n support
  - Active maintenance by Meta

- **Content Format**: Hybrid Markdown (90%) + MDX (10%)
  - Pure Markdown for Modules 1-3 (foundational content)
  - MDX for Module 4 and Capstone (interactive VLA demos)
  - Mermaid diagrams inline (native v3 support)

- **Navigation**: Manual sidebar configuration
  - Controlled learning path progression
  - Module-based categorization with `_category_.json`
  - Collapsed advanced modules by default

- **Code Presentation**: Prism React Renderer
  - Syntax highlighting for Python, Bash, XML, YAML, C++
  - Line highlighting and copy buttons
  - Title/filename support for code blocks

- **Deployment**: GitHub Pages
  - Free hosting for open-source projects
  - Automatic builds via GitHub Actions
  - Custom domain support

## Consequences

### Positive

- **Author Simplicity**: 90% of content is plain Markdown—no React knowledge required
- **Reader Experience**: Clean, fast-loading pages (<3s load time target)
- **Native Mermaid**: Diagrams render without plugins; fallback to PNG if needed
- **Version Control Friendly**: All content is plaintext Markdown in Git
- **Zero Infrastructure Cost**: GitHub Pages hosting is free

### Negative

- **MDX Learning Curve**: Content authors need React basics for Module 4 interactive components
- **Build Complexity**: Docusaurus build can fail silently on MDX syntax errors
- **Limited Interactivity**: No server-side execution; code examples are copy-paste only
- **Single Language**: English only (i18n deferred to future version)

## Alternatives Considered

**Alternative Stack A: Pure Markdown (MkDocs + Material)**
- Simpler authoring, Python ecosystem
- Why rejected: No interactive components support, less customizable

**Alternative Stack B: Pure MDX Throughout**
- Maximum interactivity, component reuse
- Why rejected: Overhead for simple content, maintenance burden, slower builds

**Alternative Stack C: GitBook**
- WYSIWYG editing, collaboration features
- Why rejected: Vendor lock-in, limited custom component support, cost for advanced features

**Alternative Stack D: Notion + Super/Fruition Export**
- Easy editing, real-time collaboration
- Why rejected: Not designed for technical documentation, code block limitations, SEO issues

**Alternative Stack E: Jupyter Book**
- Executable notebooks, Python ecosystem
- Why rejected: Heavy infrastructure, poor for non-Python content, complex deployment

## References

- Feature Spec: [specs/001-physical-ai-ebook/spec.md](../../specs/001-physical-ai-ebook/spec.md)
- Implementation Plan: [specs/001-physical-ai-ebook/plan.md](../../specs/001-physical-ai-ebook/plan.md)
- Research Document: [specs/001-physical-ai-ebook/research.md](../../specs/001-physical-ai-ebook/research.md)
- Quickstart Guide: [specs/001-physical-ai-ebook/quickstart.md](../../specs/001-physical-ai-ebook/quickstart.md)
- Related ADRs: ADR-0001 (Simulation Stack), ADR-0002 (VLA Pipeline)
- Official Docs: [Docusaurus v3](https://docusaurus.io/docs/)
