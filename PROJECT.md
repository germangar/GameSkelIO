# AI Agent Project Guide: GameSkelIO

This document is technical context for AI agents to understand the architecture, data flow, and conventions of the GameSkelIO project.

## 1. Project Overview
- **Type**: C Library (`libgameskelio.a`) with a pure C CLI tool (`main.c`).
- **Purpose**: High-performance 3D skeletal model and animation transcoding.
- **Architecture**: **Memory-First**. Loaders and writers operate primarily on buffers (`void* data, size_t size`) to support Virtual File Systems (VFS) and minimize I/O overhead.

## 2. Core Technical Context
- **Coordinate System**: Native **Y-Up, Right-Handed**.
- **Winding Order**: Native **Counter-Clockwise (CCW)**.
- **Animation Style**: **Sparse, timestamp-based** (seconds). 
- **Frame-Agnostic Core**: The internal IR (`gs_model`) does NOT store FPS or frame indices. All "baking" to fixed frames (like IQM's 30 FPS) is handled locally within format-specific loaders/writers.
- **Rotation Handling**: Critical. Uses quaternions (xyzw) and a "Continuous Exact Alias Solver" in `math_utils.h` to prevent Euler flipping.

## 3. Data Hierarchy (Public C API)
- **`gs_model`**: The root container.
- **`gs_animation`**: Flattened clip. Contains `num_bones` and `gs_bone_anim* bones`.
- **`gs_bone_anim`**: Group of channels (`translation`, `rotation`, `scale`) for one joint.
- **`gs_anim_channel`**: Raw sparse track. Contains `num_keys`, `double* times`, and `float* values`.
- **`gs_legacy_framegroup`**: Metadata struct for format overrides (name, first_frame, num_frames, fps).

## 4. Format-Specific Logic
- **IQM/SKM Loaders**: Handle Z-up to Y-up conversion. Support `gs_legacy_framegroup` overrides during loading to integrate external `animation.cfg` metadata into the sparse IR.
- **IQM Writer**:
    - **Transcoder Mode**: Optionally returns calculated metadata (frame ranges) during binary export.
    - **Single Animation**: Supports forcing all tracks into one continuous animation stack (useful for QFusion compatibility).
    - **Baking**: Standardizes on `BASE_FPS` (30.0) and uses **linear duration snapping** to align baked frames perfectly with track ends.
- **GLB/FBX**: Support sparse ingestion. FBX is currently file-export only due to third-party writer constraints.

## 5. Component Responsibilities
- **`gameskelio.h`**: The public C API. **Main reference for data structures.**
- **`gameskelio.cpp`**: C++/C bridge. Handles deep copying between the internal C++ `Model` and C `gs_model`.
- **`model.h`**: The internal C++ "Source of Truth" representation.
- **`main.c`**: Reference implementation showing how to use the library as a transcoder (Buffer loading -> Buffer export).

## 6. Critical Pitfalls for AI
- **Memory Management**: The library performs deep allocations. You **MUST** call `gsk_free_model(gs_model*)` and `gsk_free_buffer(void*)` to avoid leaks.
- **Coordinate Integrity**: When editing IQM/SKM loaders, ensure the root-joint and vertex Z-to-Y swaps match the committed standard exactly (see `git show HEAD:iqm_loader.cpp`).
- **Metadata Alignment**: The `gs_legacy_framegroup` provided during export is calculated *during* the bake process to guarantee the `.cfg` matches the binary buffer.

## 7. Build Workflow
- **`make`**: Builds library and tool in the root.
- **Linkage**: Use a C++ linker (`g++`) even for the C tool to resolve internal C++ dependencies.
