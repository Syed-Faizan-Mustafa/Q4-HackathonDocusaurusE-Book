# Tasks: Physical AI & Humanoid Robotics E-Book

**Input**: Design documents from `/specs/001-physical-ai-ebook/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **E-Book Source**: `my-website/docs/` for content
- **Components**: `my-website/src/components/` for React components
- **Static Assets**: `my-website/static/` for images and downloads
- **Configuration**: `my-website/` root for Docusaurus config

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and Docusaurus configuration

- [ ] T001 Initialize Docusaurus v3.9.2 project in `my-website/` with `npx create-docusaurus@latest my-website classic --typescript`
- [ ] T002 Configure `my-website/docusaurus.config.ts` with site title "Physical AI & Humanoid Robotics", tagline, and GitHub Pages settings
- [ ] T003 [P] Configure Mermaid diagram support in `my-website/docusaurus.config.ts` using `@docusaurus/theme-mermaid`
- [ ] T004 [P] Configure Prism syntax highlighting for Python, Bash, XML, YAML, C++ in `my-website/docusaurus.config.ts`
- [ ] T005 [P] Create custom theme colors for robotics theme in `my-website/src/css/custom.css` (dark blue/orange palette)
- [ ] T006 Verify Docusaurus builds with `npm run build` in `my-website/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Configure manual sidebar structure in `my-website/sidebars.ts` with 5 module categories and appendix
- [ ] T008 [P] Create module directory structure: `my-website/docs/module-1-ros2/`, `module-2-gazebo/`, `module-3-isaac/`, `module-4-vla/`, `module-5-hardware/`, `capstone/`, `appendix/`
- [ ] T009 [P] Create `_category_.json` files for each module directory with position, label, and collapsed settings
- [ ] T010 [P] Create `my-website/static/img/mermaid-fallbacks/` directory for PNG diagram fallbacks
- [ ] T011 [P] Create `my-website/static/downloads/` directory for code snippet downloads
- [ ] T012 Create landing page `my-website/docs/intro.md` with e-book overview, target audience, and learning path summary
- [ ] T013 [P] Create prerequisites page `my-website/docs/prerequisites.md` with Ubuntu 22.04, ROS 2 Humble, GPU requirements
- [ ] T014 [P] Create learning path page `my-website/docs/learning-path.md` with Mermaid diagram showing module progression
- [ ] T015 Verify sidebar navigation renders correctly with `npm run start`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Module Navigation and Learning Flow (Priority: P1) 🎯 MVP

**Goal**: Developer navigates through structured learning path from ROS 2 fundamentals to VLA concepts

**Independent Test**: Load Docusaurus site, navigate between modules and sections, verify all content renders correctly

### Module 1: ROS 2 Fundamentals (9 chapters)

- [ ] T016 [P] [US1] Create `my-website/docs/module-1-ros2/01-intro.md` with module overview, learning objectives, and chapter list
- [ ] T017 [P] [US1] Create `my-website/docs/module-1-ros2/02-concepts.md` with ROS 2 architecture explanation and 3 Mermaid diagrams (nodes, topics, services)
- [ ] T018 [P] [US1] Create `my-website/docs/module-1-ros2/03-setup.md` with ROS 2 Humble installation guide for Ubuntu 22.04
- [ ] T019 [P] [US1] Create `my-website/docs/module-1-ros2/04-hello-ros2.md` with first Python node example (rclpy)
- [ ] T020 [P] [US1] Create `my-website/docs/module-1-ros2/05-pubsub.md` with publisher/subscriber pattern and launch file
- [ ] T021 [P] [US1] Create `my-website/docs/module-1-ros2/06-services.md` with request/response pattern (client + server)
- [ ] T022 [P] [US1] Create `my-website/docs/module-1-ros2/07-actions.md` with goal-based task pattern (action server + client)
- [ ] T023 [P] [US1] Create `my-website/docs/module-1-ros2/08-urdf-basics.md` with URDF humanoid modeling fundamentals
- [ ] T024 [P] [US1] Create `my-website/docs/module-1-ros2/09-troubleshooting.md` with common ROS 2 issues and solutions

### Module 2: Digital Twins with Gazebo (7 chapters)

- [ ] T025 [P] [US1] Create `my-website/docs/module-2-gazebo/01-intro.md` with simulation overview and learning objectives
- [ ] T026 [P] [US1] Create `my-website/docs/module-2-gazebo/02-gazebo-basics.md` with Gazebo Harmonic installation and first launch
- [ ] T027 [P] [US1] Create `my-website/docs/module-2-gazebo/03-worlds.md` with SDF world file creation and customization
- [ ] T028 [P] [US1] Create `my-website/docs/module-2-gazebo/04-urdf-to-sim.md` with spawning URDF robots in Gazebo
- [ ] T029 [P] [US1] Create `my-website/docs/module-2-gazebo/05-physics-tuning.md` with physics engine parameters and tuning
- [ ] T030 [P] [US1] Create `my-website/docs/module-2-gazebo/06-plugins.md` with sensor plugin integration (camera, lidar, IMU)
- [ ] T031 [P] [US1] Create `my-website/docs/module-2-gazebo/07-troubleshooting.md` with common Gazebo issues and solutions

### Module 3: Isaac Sim & Navigation (7 chapters)

- [ ] T032 [P] [US1] Create `my-website/docs/module-3-isaac/01-intro.md` with Isaac ecosystem overview and RTX requirements
- [ ] T033 [P] [US1] Create `my-website/docs/module-3-isaac/02-isaac-setup.md` with Omniverse and Isaac Sim installation
- [ ] T034 [P] [US1] Create `my-website/docs/module-3-isaac/03-photorealistic.md` with photorealistic scene setup and ray tracing
- [ ] T035 [P] [US1] Create `my-website/docs/module-3-isaac/04-isaac-ros.md` with Isaac ROS package integration
- [ ] T036 [P] [US1] Create `my-website/docs/module-3-isaac/05-vslam.md` with visual SLAM configuration and usage
- [ ] T037 [P] [US1] Create `my-website/docs/module-3-isaac/06-nav2.md` with Nav2 integration for bipedal locomotion
- [ ] T038 [P] [US1] Create `my-website/docs/module-3-isaac/07-troubleshooting.md` with common Isaac issues and solutions

### Module 4: VLA & AI Brain (5 chapters)

- [ ] T039 [P] [US1] Create `my-website/docs/module-4-vla/01-intro.md` with VLA pipeline overview and learning objectives
- [ ] T040 [P] [US1] Create `my-website/docs/module-4-vla/02-whisper.mdx` with Whisper voice recognition integration (interactive demo)
- [ ] T041 [P] [US1] Create `my-website/docs/module-4-vla/03-llm-planner.mdx` with Claude/Llama task planner integration
- [ ] T042 [P] [US1] Create `my-website/docs/module-4-vla/04-vla-pipeline.mdx` with full Voice→LLM→ROS 2 integration
- [ ] T043 [P] [US1] Create `my-website/docs/module-4-vla/05-troubleshooting.md` with VLA pipeline debugging guide

### Module 5: Hardware & Deployment (5 chapters)

- [ ] T044 [P] [US1] Create `my-website/docs/module-5-hardware/01-intro.md` with hardware stack overview
- [ ] T045 [P] [US1] Create `my-website/docs/module-5-hardware/02-workstation-specs.md` with RTX workstation requirements and GPU verification
- [ ] T046 [P] [US1] Create `my-website/docs/module-5-hardware/03-jetson-deployment.md` with Jetson Orin flash and ROS 2 deployment
- [ ] T047 [P] [US1] Create `my-website/docs/module-5-hardware/04-sensor-integration.md` with RealSense, IMU, and microphone setup
- [ ] T048 [P] [US1] Create `my-website/docs/module-5-hardware/05-troubleshooting.md` with hardware debugging guide

### Appendix (4 chapters)

- [ ] T049 [P] [US1] Create `my-website/docs/appendix/glossary.md` with Physical AI and ROS 2 terminology definitions
- [ ] T050 [P] [US1] Create `my-website/docs/appendix/ros2-cli-reference.md` with common ROS 2 command reference
- [ ] T051 [P] [US1] Create `my-website/docs/appendix/troubleshooting.md` with cross-module troubleshooting matrix
- [ ] T052 [P] [US1] Create `my-website/docs/appendix/resources.md` with external links to official documentation (APA formatted)

### Navigation Validation

- [ ] T053 [US1] Test navigation between all modules via sidebar in browser
- [ ] T054 [US1] Verify `npm run build` produces zero errors and zero warnings
- [ ] T055 [US1] Test mobile responsive navigation layout

**Checkpoint**: User Story 1 complete - all 35+ chapters navigable, site builds successfully

---

## Phase 4: User Story 2 - Running Code Examples (Priority: P1) 🎯 MVP

**Goal**: Developer copies code examples and executes them on Ubuntu 22.04 successfully

**Independent Test**: Follow any code example on Ubuntu 22.04 with prerequisites, verify expected output

### Module 1: ROS 2 Code Examples (~19 examples)

- [ ] T056 [P] [US2] Add 5 installation command examples to `my-website/docs/module-1-ros2/03-setup.md` with copy buttons
- [ ] T057 [P] [US2] Add 2 Python node examples to `my-website/docs/module-1-ros2/04-hello-ros2.md` with expected output
- [ ] T058 [P] [US2] Add 3 publisher/subscriber examples to `my-website/docs/module-1-ros2/05-pubsub.md` with launch file
- [ ] T059 [P] [US2] Add 2 service examples to `my-website/docs/module-1-ros2/06-services.md` with client/server code
- [ ] T060 [P] [US2] Add 3 action examples to `my-website/docs/module-1-ros2/07-actions.md` with action server/client
- [ ] T061 [P] [US2] Add 4 URDF examples to `my-website/docs/module-1-ros2/08-urdf-basics.md` with humanoid joints

### Module 2: Gazebo Code Examples (~16 examples)

- [ ] T062 [P] [US2] Add 4 installation/launch examples to `my-website/docs/module-2-gazebo/02-gazebo-basics.md`
- [ ] T063 [P] [US2] Add 3 SDF world file examples to `my-website/docs/module-2-gazebo/03-worlds.md`
- [ ] T064 [P] [US2] Add 2 spawn command examples to `my-website/docs/module-2-gazebo/04-urdf-to-sim.md`
- [ ] T065 [P] [US2] Add 3 physics parameter examples to `my-website/docs/module-2-gazebo/05-physics-tuning.md`
- [ ] T066 [P] [US2] Add 4 sensor plugin examples to `my-website/docs/module-2-gazebo/06-plugins.md`

### Module 3: Isaac Sim Code Examples (~19 examples)

- [ ] T067 [P] [US2] Add 5 Omniverse install examples to `my-website/docs/module-3-isaac/02-isaac-setup.md`
- [ ] T068 [P] [US2] Add 3 scene setup examples to `my-website/docs/module-3-isaac/03-photorealistic.md`
- [ ] T069 [P] [US2] Add 4 Isaac ROS package examples to `my-website/docs/module-3-isaac/04-isaac-ros.md`
- [ ] T070 [P] [US2] Add 3 VSLAM command examples to `my-website/docs/module-3-isaac/05-vslam.md`
- [ ] T071 [P] [US2] Add 4 Nav2 configuration examples to `my-website/docs/module-3-isaac/06-nav2.md`

### Module 4: VLA Code Examples (~15 examples)

- [ ] T072 [P] [US2] Add 4 Whisper integration examples to `my-website/docs/module-4-vla/02-whisper.mdx`
- [ ] T073 [P] [US2] Add 5 LLM planner examples (Claude API + Llama local) to `my-website/docs/module-4-vla/03-llm-planner.mdx`
- [ ] T074 [P] [US2] Add 6 full VLA pipeline examples to `my-website/docs/module-4-vla/04-vla-pipeline.mdx`

### Module 5: Hardware Code Examples (~11 examples)

- [ ] T075 [P] [US2] Add 2 GPU verification examples to `my-website/docs/module-5-hardware/02-workstation-specs.md`
- [ ] T076 [P] [US2] Add 5 Jetson flash/deploy examples to `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T077 [P] [US2] Add 4 sensor integration examples to `my-website/docs/module-5-hardware/04-sensor-integration.md`

### Code Example Validation

- [ ] T078 [US2] Verify all Module 1 code examples execute on fresh Ubuntu 22.04 with ROS 2 Humble
- [ ] T079 [US2] Verify all Module 2 code examples execute with Gazebo Harmonic installed
- [ ] T080 [US2] Verify all Module 3 code examples (mark RTX-required examples clearly)
- [ ] T081 [US2] Verify all Module 4 code examples with Whisper and LLM access
- [ ] T082 [US2] Verify all Module 5 code examples with appropriate hardware

**Checkpoint**: User Story 2 complete - ~80 code examples with prerequisites, expected outputs, and copy buttons

---

## Phase 5: User Story 3 - Simulation Environment Setup (Priority: P2)

**Goal**: Developer sets up Gazebo and/or Isaac Sim to run physics simulation with humanoid robot

**Independent Test**: Follow simulation setup guide on RTX workstation and launch sample simulation scene

### Gazebo Setup Enhancement

- [ ] T083 [P] [US3] Add detailed GPU driver requirements to `my-website/docs/module-2-gazebo/02-gazebo-basics.md`
- [ ] T084 [P] [US3] Add step-by-step Gazebo Harmonic installation verification to `my-website/docs/module-2-gazebo/02-gazebo-basics.md`
- [ ] T085 [P] [US3] Create sample humanoid URDF model file in `my-website/static/downloads/humanoid_basic.urdf`
- [ ] T086 [US3] Add end-to-end "First Simulation" tutorial to `my-website/docs/module-2-gazebo/04-urdf-to-sim.md`

### Isaac Sim Setup Enhancement

- [ ] T087 [P] [US3] Add exact RTX driver version requirements to `my-website/docs/module-3-isaac/02-isaac-setup.md`
- [ ] T088 [P] [US3] Add Omniverse Launcher installation walkthrough to `my-website/docs/module-3-isaac/02-isaac-setup.md`
- [ ] T089 [P] [US3] Add Isaac Sim license verification steps to `my-website/docs/module-3-isaac/02-isaac-setup.md`
- [ ] T090 [US3] Add end-to-end "First Isaac Scene" tutorial to `my-website/docs/module-3-isaac/03-photorealistic.md`

### Cloud GPU Alternative

- [ ] T091 [P] [US3] Add AWS g5/g6 cloud GPU setup guide to `my-website/docs/prerequisites.md`
- [ ] T092 [US3] Add cost comparison and recommendations for cloud vs. local GPU

**Checkpoint**: User Story 3 complete - both Gazebo and Isaac Sim setup paths fully documented

---

## Phase 6: User Story 4 - Understanding Diagrams and Architecture (Priority: P2)

**Goal**: Educator/learner views architecture diagrams to understand VLA pipeline flow

**Independent Test**: View any Mermaid diagram in Docusaurus site, verify it renders and conveys intended concept

### Mermaid Diagrams (~20 total)

- [ ] T093 [P] [US4] Create learning path Mermaid diagram in `my-website/docs/module-1-ros2/01-intro.md`
- [ ] T094 [P] [US4] Create ROS 2 node architecture diagram in `my-website/docs/module-1-ros2/02-concepts.md`
- [ ] T095 [P] [US4] Create ROS 2 topic pub/sub diagram in `my-website/docs/module-1-ros2/02-concepts.md`
- [ ] T096 [P] [US4] Create ROS 2 service diagram in `my-website/docs/module-1-ros2/02-concepts.md`
- [ ] T097 [P] [US4] Create pub/sub flow diagram in `my-website/docs/module-1-ros2/05-pubsub.md`
- [ ] T098 [P] [US4] Create service sequence diagram in `my-website/docs/module-1-ros2/06-services.md`
- [ ] T099 [P] [US4] Create action state machine diagram in `my-website/docs/module-1-ros2/07-actions.md`
- [ ] T100 [P] [US4] Create URDF joint hierarchy diagram in `my-website/docs/module-1-ros2/08-urdf-basics.md`
- [ ] T101 [P] [US4] Create simulation overview diagram in `my-website/docs/module-2-gazebo/01-intro.md`
- [ ] T102 [P] [US4] Create SDF world structure diagram in `my-website/docs/module-2-gazebo/03-worlds.md`
- [ ] T103 [P] [US4] Create URDF to SDF pipeline diagram in `my-website/docs/module-2-gazebo/04-urdf-to-sim.md`
- [ ] T104 [P] [US4] Create Gazebo plugin architecture diagram in `my-website/docs/module-2-gazebo/06-plugins.md`
- [ ] T105 [P] [US4] Create Isaac ecosystem diagram in `my-website/docs/module-3-isaac/01-intro.md`
- [ ] T106 [P] [US4] Create rendering pipeline diagram in `my-website/docs/module-3-isaac/03-photorealistic.md`
- [ ] T107 [P] [US4] Create Isaac ROS stack diagram in `my-website/docs/module-3-isaac/04-isaac-ros.md`
- [ ] T108 [P] [US4] Create VSLAM flow diagram in `my-website/docs/module-3-isaac/05-vslam.md`
- [ ] T109 [P] [US4] Create Nav2 architecture diagram in `my-website/docs/module-3-isaac/06-nav2.md`
- [ ] T110 [P] [US4] Create VLA pipeline overview diagram in `my-website/docs/module-4-vla/01-intro.md`
- [ ] T111 [P] [US4] Create voice pipeline diagram in `my-website/docs/module-4-vla/02-whisper.mdx`
- [ ] T112 [P] [US4] Create LLM planning flow diagram in `my-website/docs/module-4-vla/03-llm-planner.mdx`
- [ ] T113 [P] [US4] Create end-to-end VLA diagram in `my-website/docs/module-4-vla/04-vla-pipeline.mdx`
- [ ] T114 [P] [US4] Create hardware stack diagram in `my-website/docs/module-5-hardware/01-intro.md`
- [ ] T115 [P] [US4] Create edge architecture diagram in `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T116 [P] [US4] Create sensor stack diagram in `my-website/docs/module-5-hardware/04-sensor-integration.md`

### Fallback Image Generation

- [ ] T117 [US4] Generate PNG fallbacks for all Mermaid diagrams in `my-website/static/img/mermaid-fallbacks/`
- [ ] T118 [US4] Add fallback image references to each diagram location
- [ ] T119 [US4] Verify all diagrams render in Chrome, Firefox, Safari, and Edge browsers

**Checkpoint**: User Story 4 complete - ~20 Mermaid diagrams with PNG fallbacks

---

## Phase 7: User Story 5 - Capstone Project Completion (Priority: P3)

**Goal**: Developer builds capstone Voice → Whisper → LLM → ROS 2 → Robot Action pipeline

**Independent Test**: Follow capstone guide to integrate all components and demonstrate working pipeline in simulation

### Capstone Chapters (4 chapters + overview)

- [ ] T120 [P] [US5] Create `my-website/docs/capstone/00-overview.mdx` with progress tracker component and full system diagram
- [ ] T121 [P] [US5] Create `my-website/docs/capstone/01-integration.md` with step-by-step component integration (6 code examples)
- [ ] T122 [P] [US5] Create `my-website/docs/capstone/02-testing.md` with pipeline testing strategies (4 test commands)
- [ ] T123 [P] [US5] Create `my-website/docs/capstone/03-deployment.md` with simulation deployment guide (3 deploy scripts)
- [ ] T124 [P] [US5] Create `my-website/docs/capstone/04-demo-scripts.md` with demo voice commands and expected behaviors

### Capstone Integration

- [ ] T125 [US5] Create capstone component integration diagram in `my-website/docs/capstone/01-integration.md`
- [ ] T126 [US5] Verify capstone prerequisites link to correct module chapters
- [ ] T127 [US5] Add estimated completion time (4 hours) to `my-website/docs/capstone/00-overview.mdx`
- [ ] T128 [US5] Add milestone checkpoints to track progress in capstone

### Capstone Validation

- [ ] T129 [US5] Verify capstone pipeline conceptually works: Voice input → Whisper transcription → LLM action plan → ROS 2 dispatch
- [ ] T130 [US5] Add troubleshooting section for common capstone integration issues

**Checkpoint**: User Story 5 complete - capstone project documented with 4-hour completion path

---

## Phase 8: User Story 6 - Edge Deployment Guidance (Priority: P3)

**Goal**: Developer deploys ROS 2 application to Jetson Orin following hardware deployment guide

**Independent Test**: Follow Jetson deployment guide and run ROS 2 node on edge device

### Jetson Deployment Enhancement

- [ ] T131 [P] [US6] Add Jetson Orin Nano/NX model specifications to `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T132 [P] [US6] Add JetPack SDK installation guide to `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T133 [P] [US6] Add ROS 2 Humble cross-compilation guide to `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T134 [US6] Add end-to-end "First Edge Node" deployment tutorial

### Sensor Integration for Edge

- [ ] T135 [P] [US6] Add RealSense D435i setup for Jetson to `my-website/docs/module-5-hardware/04-sensor-integration.md`
- [ ] T136 [P] [US6] Add IMU integration guide to `my-website/docs/module-5-hardware/04-sensor-integration.md`
- [ ] T137 [P] [US6] Add USB microphone setup for voice input to `my-website/docs/module-5-hardware/04-sensor-integration.md`

### Edge Performance

- [ ] T138 [US6] Add Whisper-tiny optimization guide for Jetson to `my-website/docs/module-5-hardware/03-jetson-deployment.md`
- [ ] T139 [US6] Add latency benchmarks and expectations for edge deployment

**Checkpoint**: User Story 6 complete - Jetson deployment path with sensor integration

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### APA Citations

- [ ] T140 [P] Verify all ROS 2 documentation citations in APA format across modules
- [ ] T141 [P] Verify all Gazebo documentation citations in APA format
- [ ] T142 [P] Verify all Isaac Sim documentation citations in APA format
- [ ] T143 [P] Verify all Whisper and LLM documentation citations in APA format
- [ ] T144 Add APA formatted references section to `my-website/docs/appendix/resources.md`

### Content Quality

- [ ] T145 [P] Verify Flesch-Kincaid readability grade 9-12 for all chapters
- [ ] T146 Run spellcheck across all content files
- [ ] T147 Verify all external links are working (link rot check)

### Build & Deployment

- [ ] T148 Run final `npm run build` and verify zero errors, zero warnings
- [ ] T149 Configure GitHub Actions workflow for automated deployment in `.github/workflows/deploy.yml`
- [ ] T150 Deploy to GitHub Pages and verify all pages accessible
- [ ] T151 Verify page load time <3 seconds on standard broadband

### Final Validation

- [ ] T152 Run quickstart.md validation - author can create new chapter in <5 minutes
- [ ] T153 Verify mobile responsive layout across all modules
- [ ] T154 Test search functionality for key robotics terms

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel (P1 priority)
  - US3 and US4 can proceed after US1 structure exists (P2 priority)
  - US5 and US6 depend on prior modules existing (P3 priority)
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

- **US1 (P1)**: Can start after Foundational (Phase 2) - Creates chapter structure
- **US2 (P1)**: Can start after US1 creates chapter files - Adds code examples
- **US3 (P2)**: Enhances existing Gazebo/Isaac chapters from US1
- **US4 (P2)**: Adds diagrams to existing chapters from US1
- **US5 (P3)**: Requires Modules 1-4 structure from US1
- **US6 (P3)**: Requires Module 5 structure from US1

### Within Each User Story

- Chapters can be created in parallel [P] within the same module
- Code examples [P] can be added in parallel to different chapters
- Diagrams [P] can be created in parallel for different chapters
- Validation tasks must run after content is complete

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, US1 and US2 can start in parallel
- All chapter creation tasks within US1 are marked [P]
- All code example tasks within US2 are marked [P]
- All diagram tasks within US4 are marked [P]

---

## Implementation Strategy

### MVP First (US1 + US2 Only)

1. Complete Phase 1: Setup (Docusaurus initialized)
2. Complete Phase 2: Foundational (sidebar, directories, landing pages)
3. Complete Phase 3: US1 - All 35+ chapters with content
4. Complete Phase 4: US2 - All ~80 code examples
5. **STOP and VALIDATE**: Build succeeds, navigation works, code examples have prerequisites/outputs
6. Deploy/demo if ready (MVP!)

### Incremental Delivery

1. Setup + Foundational → E-book skeleton ready
2. Add US1 → Navigable modules with content → Deploy (MVP!)
3. Add US2 → Working code examples → Deploy
4. Add US3 + US4 → Enhanced simulation guides + diagrams → Deploy
5. Add US5 + US6 → Capstone + edge deployment → Deploy (Full product)
6. Add Polish → Production quality → Final release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: US1 - Module 1-2 chapters
   - Developer B: US1 - Module 3-5 chapters
   - Developer C: US2 - Code examples for Module 1-2
   - Developer D: US4 - Mermaid diagrams
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All code examples must include: prerequisites, expected output, copy button
- All diagrams must have PNG fallbacks for browser compatibility
- APA citation style required for all external references
