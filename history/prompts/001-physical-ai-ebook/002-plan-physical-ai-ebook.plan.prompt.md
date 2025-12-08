---
id: 002
title: Plan Physical AI E-Book Architecture
stage: plan
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-ebook
branch: 001-physical-ai-ebook
user: faizanmustafa
command: /sp.plan
labels: ["physical-ai", "ebook", "architecture", "ros2", "gazebo", "isaac", "vla"]
links:
  spec: specs/001-physical-ai-ebook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-ebook/plan.md
  - specs/001-physical-ai-ebook/research.md
  - specs/001-physical-ai-ebook/data-model.md
  - specs/001-physical-ai-ebook/quickstart.md
  - specs/001-physical-ai-ebook/contracts/vla-ros2-interfaces.md
  - CLAUDE.md
tests:
  - N/A (planning phase)
---

## Prompt

AI/Spec-Driven E-Book on Physical AI & Humanoid Robotics

Create:
- Architecture sketch for full E-Book structure (Docusaurus layout, modules, build flow)
- Section hierarchy based on 4 learning modules + hardware + capstone
- Research approach aligned with robotics documentation (ROS 2, Gazebo, Unity, NVIDIA Isaac, Whisper, VLA)
- Quality validation checklist (accuracy, reproducibility, APA citations, Docusaurus build success)

Decisions needing documentation:
1. **Simulation stack choice**
   - Options: Gazebo Classic | Gazebo Garden | Unity | Isaac Sim
   - Tradeoffs: realism, performance, hardware requirements

2. **Hardware environment**
   - Local RTX workstation vs. Cloud GPU (AWS g5/g6)
   - Tradeoffs: cost, latency, reliability

3. **Robot platform**
   - Unitree Go2 | Unitree G1 | OP3 | Proxy robots (arms, quadrupeds)
   - Tradeoffs: cost vs. humanoid fidelity

4. **VLA integration path**
   - Whisper + LLM planning → ROS 2 actions
   - Options: OpenAI, local models, cloud inference
   - Tradeoffs: latency, accuracy, hardware load

5. **Documentation structure**
   - MDX vs. pure Markdown
   - Tradeoffs: interactivity vs. simplicity

Testing strategy:
- Validate commands by running them on Ubuntu 22.04
- Verify each ROS 2, Gazebo, Unity, and Isaac workflow reproduces exact results
- Check Docusaurus compiles without errors (`npm run start`)
- Confirm diagrams, code blocks, and MDX components render correctly
- Ensure APA citation compliance from Constitution
- Run consistency checks for module ordering and learning outcomes
- End-to-end test: Capstone pipeline must function conceptually (Voice → Whisper → LLM → ROS 2 → Navigation → Perception → Manipulation)

Technical details:
- Use **research-concurrent writing** (research while writing, not upfront)
- Follow APA citation style defined in Constitution
- Organize the writing plan by phases:
  1. **Research**
  2. **Foundation**
  3. **Analysis**
  4. **Synthesis**

## Response snapshot

Created comprehensive implementation plan with 5 key architectural decisions:

**ADR-001: Simulation Stack** - Gazebo Harmonic (primary) + Isaac Sim (advanced)
- Gazebo: Native ROS 2 integration, accessible hardware, industry standard
- Isaac: Optional for RTX users wanting photorealistic rendering

**ADR-002: VLA Pipeline** - Whisper (local) + Claude/Llama (hybrid) + ROS 2 Executor
- Hybrid approach with documented cloud and local alternatives
- Latency budget: <3 seconds end-to-end

**ADR-003: Documentation Format** - Markdown (90%) + MDX (10%)
- MDX only for interactive VLA demos (Module 4, Capstone)
- Manual sidebar configuration for controlled learning path

**Artifacts Generated**:
- `research.md` - 5 technology decisions with rationale
- `data-model.md` - Content entities (Module, Chapter, CodeExample, Diagram)
- `quickstart.md` - Author development guide
- `contracts/vla-ros2-interfaces.md` - ROS 2 message/service definitions
- `plan.md` - Complete implementation plan with project structure

**Project Structure**: 5 modules, 35 chapters, ~80 code examples, ~20 diagrams

## Outcome

- ✅ Impact: Created complete implementation plan ready for task generation
- 🧪 Tests: N/A (planning phase - architecture validation only)
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/ created
- 🔁 Next prompts: /sp.tasks (to generate implementation task list)
- 🧠 Reflection: Research agents provided valuable context for technology decisions. Constitution checks all pass.

## Evaluation notes (flywheel)

- Failure modes observed: None - all research agents completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (5/5 principles)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to generate atomic writing tasks

---

📋 **Architectural decisions detected**: 3 significant decisions documented inline in plan.md
- ADR-001: Simulation Stack Selection (Gazebo Harmonic + Isaac Sim)
- ADR-002: VLA Pipeline Architecture (Whisper + Claude/Llama + ROS 2)
- ADR-003: Documentation Format (Hybrid Markdown + MDX)

Document reasoning and tradeoffs? Run `/sp.adr <decision-title>` for formal ADR creation.
