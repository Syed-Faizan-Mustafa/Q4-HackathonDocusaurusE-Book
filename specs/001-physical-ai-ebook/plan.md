# Implementation Plan: Physical AI & Humanoid Robotics E-Book

**Branch**: `001-physical-ai-ebook` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-ebook/spec.md`

## Summary

Build a modular, interactive e-book teaching Physical AI development from ROS 2 fundamentals to Vision-Language-Action (VLA) systems for humanoid robots. The e-book uses Docusaurus v3 with Markdown/MDX, targets Ubuntu 22.04 with ROS 2 Humble, and culminates in a capstone project demonstrating Voice → Whisper → LLM → ROS 2 → Robot Action pipeline.

**Primary Simulation**: Gazebo Harmonic (native ROS 2 integration, accessible hardware requirements)
**Advanced Option**: NVIDIA Isaac Sim (photorealistic rendering, RTX required)
**VLA Stack**: Whisper (local) + Claude 3.5/Llama 3.1 (cloud/local) + Custom ROS 2 action executor

## Technical Context

**Language/Version**: Markdown/MDX (Docusaurus v3.9.2), Python 3.10+ (rclpy), Bash
**Primary Dependencies**: Docusaurus 3.9.2, @mdx-js/react 3.0, ROS 2 Humble, Gazebo Harmonic, Whisper, Claude API/Llama
**Storage**: Git (content), GitHub Pages (deployment), static assets in `/static/`
**Testing**: `npm run build` (Docusaurus), Ubuntu 22.04 code verification, Mermaid rendering validation
**Target Platform**: Web (GitHub Pages), Ubuntu 22.04 (code examples)
**Project Type**: Documentation/E-Book
**Performance Goals**: <3 second page load, 100% code example success rate
**Constraints**: APA citations required, official documentation sources only, no hallucinated APIs
**Scale/Scope**: 5 modules, 35 chapters, ~80 code examples, ~20 diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| **Accuracy** | All technical claims linked to authoritative sources | ✅ PASS | Research.md lists official docs |
| **Clarity** | Flesch-Kincaid grade 9-12 | ✅ PASS | Target intermediate developers |
| **Reproducibility** | Version-locked examples on Ubuntu 22.04 | ✅ PASS | ROS 2 Humble specified |
| **Consistency** | SpecKit-Plus methodology, APA citations | ✅ PASS | Template structure applied |
| **Modularity** | Designed for updates and extensions | ✅ PASS | Per-module directory structure |

**Key Standards Compliance**:
- [x] All examples run on Ubuntu-based environments
- [x] Citation style: APA
- [x] Documentation style: Markdown/MDX for Docusaurus
- [x] Zero tolerance for hallucinated APIs
- [x] Writing readability: Flesch-Kincaid grade 9-12

**Constraints Compliance**:
- [x] Platform: Docusaurus (initialized)
- [x] Workflow: SpecKit-Plus + Claude Code
- [x] Deliverable: GitHub Pages deployment
- [x] Testing: `npm run start` builds successfully

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-ebook/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 output - technology decisions
├── data-model.md        # Phase 1 output - content entities
├── quickstart.md        # Phase 1 output - author guide
├── contracts/           # Phase 1 output
│   └── vla-ros2-interfaces.md  # ROS 2 message/service contracts
├── checklists/
│   └── requirements.md  # Spec quality validation
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (Docusaurus E-Book)

```text
my-website/
├── docs/
│   ├── intro.md                    # Landing page
│   ├── prerequisites.md            # System requirements
│   ├── learning-path.md            # Visual roadmap
│   │
│   ├── module-1-ros2/              # Module 1: ROS 2 Fundamentals
│   │   ├── _category_.json
│   │   ├── 01-intro.md
│   │   ├── 02-concepts.md          # Mermaid: ROS 2 architecture
│   │   ├── 03-setup.md             # Installation guide
│   │   ├── 04-hello-ros2.md        # First node
│   │   ├── 05-pubsub.md            # Publisher/Subscriber
│   │   ├── 06-services.md          # Request/Response
│   │   ├── 07-actions.md           # Goal-based tasks
│   │   ├── 08-urdf-basics.md       # URDF for humanoids
│   │   └── 09-troubleshooting.md
│   │
│   ├── module-2-gazebo/            # Module 2: Digital Twins
│   │   ├── _category_.json
│   │   ├── 01-intro.md
│   │   ├── 02-gazebo-basics.md
│   │   ├── 03-worlds.md
│   │   ├── 04-urdf-to-sim.md
│   │   ├── 05-physics-tuning.md
│   │   ├── 06-plugins.md
│   │   └── 07-troubleshooting.md
│   │
│   ├── module-3-isaac/             # Module 3: NVIDIA Isaac
│   │   ├── _category_.json
│   │   ├── 01-intro.md
│   │   ├── 02-isaac-setup.md       # RTX requirements
│   │   ├── 03-photorealistic.md
│   │   ├── 04-isaac-ros.md
│   │   ├── 05-vslam.md
│   │   ├── 06-nav2.md
│   │   └── 07-troubleshooting.md
│   │
│   ├── module-4-vla/               # Module 4: VLA & AI Brain
│   │   ├── _category_.json
│   │   ├── 01-intro.md
│   │   ├── 02-whisper.mdx          # Interactive voice demo
│   │   ├── 03-llm-planner.mdx      # LLM integration
│   │   ├── 04-vla-pipeline.mdx     # Full pipeline
│   │   └── 05-troubleshooting.md
│   │
│   ├── module-5-hardware/          # Module 5: Hardware & Deployment
│   │   ├── _category_.json
│   │   ├── 01-intro.md
│   │   ├── 02-workstation-specs.md
│   │   ├── 03-jetson-deployment.md
│   │   ├── 04-sensor-integration.md
│   │   └── 05-troubleshooting.md
│   │
│   ├── capstone/                   # Capstone Project
│   │   ├── _category_.json
│   │   ├── 00-overview.mdx         # Progress tracker
│   │   ├── 01-integration.md
│   │   ├── 02-testing.md
│   │   ├── 03-deployment.md
│   │   └── 04-demo-scripts.md
│   │
│   └── appendix/                   # Reference Materials
│       ├── _category_.json
│       ├── glossary.md
│       ├── ros2-cli-reference.md
│       ├── troubleshooting.md
│       └── resources.md
│
├── src/
│   ├── components/
│   │   └── ModuleProgress.tsx      # Optional: capstone tracker
│   └── css/
│       └── custom.css              # Robotics theme
│
├── static/
│   ├── img/
│   │   ├── logo.svg
│   │   └── mermaid-fallbacks/      # PNG exports
│   └── downloads/                  # Code snippets, configs
│
├── docusaurus.config.ts            # Site configuration
├── sidebars.ts                     # Manual navigation
└── package.json
```

**Structure Decision**: Documentation e-book using Docusaurus v3 with module-based organization. Pure Markdown for Modules 1-3, MDX for interactive components in Modules 4-5 and Capstone.

## Complexity Tracking

No violations requiring justification. The design follows constitution principles with:
- Standard Docusaurus structure
- No custom backend or API
- No complex state management
- All interactions are documentation/content

## Architecture Decisions

### ADR-001: Simulation Stack Selection

**Decision**: Gazebo Harmonic (primary) + Isaac Sim (advanced)

**Context**: E-book needs simulation platform that works with ROS 2 Humble on Ubuntu 22.04 for educational purposes.

**Options Considered**:
1. Gazebo Classic (deprecated, not recommended)
2. Gazebo Harmonic (active, native ROS 2)
3. Unity (TCP bridge, game-dev learning curve)
4. Isaac Sim (excellent but NVIDIA-exclusive)

**Decision**: Gazebo Harmonic for primary content (accessible hardware, native ROS 2, industry standard), Isaac Sim as optional advanced track (photorealistic, RTX required).

**Rationale**: Maximizes accessibility for students while providing professional path for those with capable hardware.

### ADR-002: VLA Pipeline Architecture

**Decision**: Whisper (local) → Claude/Llama (hybrid) → Custom ROS 2 Executor

**Context**: Module 4 requires Voice-Language-Action pipeline demonstration.

**Options Considered**:
1. Cloud-only (OpenAI API throughout) - latency issues
2. Local-only (Whisper + Llama) - privacy, but complex setup
3. Hybrid (Whisper local + LLM API) - balanced

**Decision**: Hybrid approach with documented alternatives for both cloud and local deployment.

**Rationale**: Educational clarity (API simpler to demonstrate), with local option for production-minded learners.

### ADR-003: Documentation Format

**Decision**: Markdown (90%) + MDX (10%) with manual sidebar

**Context**: Balance between simplicity and interactivity.

**Options Considered**:
1. Pure Markdown - simple but static
2. Pure MDX - complex, maintenance overhead
3. Hybrid - targeted interactivity

**Decision**: Markdown for foundational content (Modules 1-3), MDX for interactive VLA demos (Module 4, Capstone).

**Rationale**: Minimizes complexity while enabling key interactive features where they add educational value.

---

## Implementation Phases

### Phase 1: Foundation (Research + Setup) ✅ COMPLETE

**Deliverables Created**:
- [x] `research.md` - Technology decisions documented
- [x] `data-model.md` - Content entity definitions
- [x] `contracts/vla-ros2-interfaces.md` - ROS 2 message contracts
- [x] `quickstart.md` - Author development guide

### Phase 2: Task Generation (Next: /sp.tasks)

**Scope**:
- Generate `tasks.md` with ordered implementation steps
- Break down each module into atomic writing tasks
- Define verification criteria per task

### Phase 3: Implementation (After /sp.tasks)

**Writing Phases** (per user requirement):

1. **Research Phase**: Gather official documentation, verify APIs, collect examples
2. **Foundation Phase**: Write Module 1 (ROS 2) and Module 2 (Gazebo) - core skills
3. **Analysis Phase**: Write Module 3 (Isaac) and Module 5 (Hardware) - advanced topics
4. **Synthesis Phase**: Write Module 4 (VLA) and Capstone - integration

### Phase 4: Validation

**Testing Strategy**:
- [x] `npm run start` - Docusaurus builds without errors
- [ ] All code examples verified on Ubuntu 22.04
- [ ] All Mermaid diagrams render correctly
- [ ] All external links working
- [ ] APA citation compliance checked
- [ ] Flesch-Kincaid readability verified
- [ ] Capstone pipeline works end-to-end in simulation

---

## Quality Validation Checklist

### Build Quality
- [ ] `npm run build` completes with zero errors
- [ ] `npm run build` completes with zero warnings
- [ ] GitHub Pages deployment successful
- [ ] All pages accessible via navigation

### Content Quality
- [ ] Each module has 3+ chapters
- [ ] Each chapter has learning objectives
- [ ] Each module has troubleshooting section
- [ ] All code examples include prerequisites
- [ ] All code examples include expected output

### Technical Accuracy
- [ ] All ROS 2 commands verified on Humble
- [ ] All Gazebo examples work on Harmonic
- [ ] All Isaac examples work with documented RTX requirements
- [ ] VLA pipeline latency within budget (<3s)

### Compliance
- [ ] APA citations for all technical claims
- [ ] Official documentation sources only
- [ ] No hallucinated APIs
- [ ] Flesch-Kincaid grade 9-12

---

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Isaac Sim license changes | Medium | Document current requirements with official links |
| ROS 2 version updates | Medium | Lock to Humble, document version requirements |
| LLM API availability | Low | Provide local Llama alternative |
| Mermaid rendering issues | Low | Generate PNG fallbacks for all diagrams |
| Code example breakage | High | Automate verification on clean Ubuntu 22.04 |

---

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Begin Module 1 (ROS 2 Fundamentals) - highest priority
3. Validate Docusaurus build after each module completion
4. Create Mermaid diagrams alongside content
5. Verify code examples on clean Ubuntu 22.04 environment

---

**Plan Status**: Ready for `/sp.tasks`
