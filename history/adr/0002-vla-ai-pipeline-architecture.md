# ADR-0002: VLA AI Pipeline Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-physical-ai-ebook
- **Context:** Module 4 of the Physical AI E-Book teaches Vision-Language-Action (VLA) integration for humanoid robots. The pipeline must demonstrate how natural language voice commands are processed through speech recognition, LLM reasoning, and ROS 2 action execution. The architecture must balance educational clarity with real-world applicability, supporting both cloud-based and local deployment scenarios.

## Decision

We will use a hybrid VLA pipeline architecture:

- **Voice Recognition**: OpenAI Whisper (local inference)
  - Model: Whisper small (244M parameters)
  - Latency: 200-300ms on RTX 3070+
  - License: MIT (no API key required)
  - Alternative: Whisper tiny for edge deployment (Jetson)

- **LLM Task Planner**: Dual-path approach
  - **Cloud Option**: Claude 3.5 Sonnet (Anthropic API)
    - Best reasoning capability for task decomposition
    - Clear documentation, excellent for teaching
    - Latency: 500-2000ms per request
  - **Local Option**: Llama 3.1 8B-Instruct
    - Privacy-focused, no data leaves machine
    - Latency: 50-100ms on RTX 3090 with quantization
    - Production-ready alternative

- **ROS 2 Integration**: Custom Action Executor
  - JSON validation layer for LLM output
  - Action client for Nav2/MoveIt integration
  - Error handling with recovery strategies

- **Latency Budget**: <3 seconds end-to-end
  - Voice capture: 500ms
  - Whisper transcription: 500ms
  - LLM planning: 1500ms
  - ROS 2 dispatch: 100ms

## Consequences

### Positive

- **Educational Flexibility**: Cloud API simplifies initial teaching; local option shows production path
- **Realistic Latency**: <3s end-to-end is acceptable for humanoid voice commands
- **Privacy Options**: Learners can choose fully local deployment with Whisper + Llama
- **Modular Design**: Each component can be upgraded independently (e.g., swap Claude for GPT-4)
- **Open Source Core**: Whisper and Llama are MIT/Meta licensed; no vendor lock-in for core components

### Negative

- **API Costs**: Claude API usage incurs costs for extended practice (~$3/$15 per 1M tokens)
- **Hardware Requirements**: Local Llama requires RTX 3070+ with 8GB VRAM minimum
- **Dual Documentation**: Must maintain both cloud and local examples
- **LLM Evolution Risk**: Rapid LLM advancement may obsolete specific model recommendations

## Alternatives Considered

**Alternative Stack A: Cloud-Only (OpenAI Throughout)**
- Whisper API + GPT-4 Turbo + function calling
- Why rejected: Higher latency (500-3000ms per component), ongoing costs, API key dependency for all examples

**Alternative Stack B: Local-Only (Whisper + Llama)**
- Maximum privacy, no API dependencies
- Why rejected: Complex setup for beginners, requires high-end GPU, less educational clarity

**Alternative Stack C: End-to-End Framework (e.g., LangChain + ROS)**
- Integrated tooling, less boilerplate
- Why rejected: Framework abstraction hides educational concepts, harder to debug

**Alternative Stack D: Pre-built VLA Models (RT-X, OpenVLA)**
- State-of-the-art research models
- Why rejected: Training complexity, not production-ready, overkill for educational context

## References

- Feature Spec: [specs/001-physical-ai-ebook/spec.md](../../specs/001-physical-ai-ebook/spec.md)
- Implementation Plan: [specs/001-physical-ai-ebook/plan.md](../../specs/001-physical-ai-ebook/plan.md)
- Research Document: [specs/001-physical-ai-ebook/research.md](../../specs/001-physical-ai-ebook/research.md)
- ROS 2 Contracts: [specs/001-physical-ai-ebook/contracts/vla-ros2-interfaces.md](../../specs/001-physical-ai-ebook/contracts/vla-ros2-interfaces.md)
- Related ADRs: ADR-0001 (Simulation Stack), ADR-0003 (Documentation)
- Official Docs: [Whisper](https://github.com/openai/whisper), [Claude API](https://docs.anthropic.com/), [Llama 3.1](https://huggingface.co/meta-llama)
