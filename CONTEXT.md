# AI Agent Project Guide: GameSkelIO

This document is technical context for AI agents to understand the architecture, data flow, and conventions of the GameSkelIO project.

## 1. Project Overview
- **Type**: C Library (`libgameskelio.a`) with a pure C CLI tool (`tools/main.c`).
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
- **IBMs as Source of Truth**: Standardized in all loaders. Every loader (IQM, GLB, FBX) MUST compute or load Inverse Bind Matrices (IBMs) into `gs_model->ibms`. This is the mandatory reference for all mesh warping.
- **IQM/SKM Loaders**: Handle Z-up to Y-up conversion. Support `gs_legacy_framegroup` overrides during loading.
- **IQM Writer**:
    - **Transcoder Mode**: Optionally returns calculated metadata (frame ranges).
    - **Single Animation**: Supports forcing all tracks into one continuous animation stack.
    - **Baking**: Standardizes on `BASE_FPS` (30.0) and uses linear duration snapping.
- **GLB/FBX**: Support sparse ingestion. FBX uses `ufbx`.

## 5. Rebinding Architecture (`gsk_rebase_pose`)
The rebind tool uses a **Mathematical Cancellation** strategy to change rest poses without breaking animations:
- **Mesh Warping**: Vertices are physically moved to a new pose (e.g., A-Pose).
- **IBM Update**: Inverse Bind Matrices are updated to match the new pose.
- ** cancellation**: Since $V_{warped} = (W_{new\_bind} \cdot IBM_{old}) \cdot V_{original}$, and $IBM_{new} = W_{new\_bind}^{-1}$, the equation $W_{anim} \cdot IBM_{new} \cdot V_{warped}$ simplifies back to $W_{anim} \cdot IBM_{old} \cdot V_{original}$.
- **Local Track Retention**: Because of this cancellation, the **Original Local TRS Keys** can be preserved 1:1, ensuring bit-perfect world-space motion.
- **Dense Bone Sync**: To prevent "Frozen Limbs", the rebind tool performs a **Full Skeleton Bake**. Every bone—even un-animated ones—receives explicit keys to force it to stay in the original world-space relative pose.

## 6. Component Responsibilities
- **`gameskelio.h`**: The public C API. Main reference for data structures.
- **`gsrebind.cpp`**: Implementation of the rebinding CLI in `tools/gsrebind.cpp`.
- **`main.c`**: Reference implementation in `tools/main.c` showing how to use the library as a transcoder.
- **`gameskelio.cpp`**: C++/C bridge. Contains `gsk_rebase_pose`, the core animation retargeter.
- **`model.h`**: Internal C++ "Source of Truth" representation.
- **`math_utils.h`**: Critical math operations (Decomposition, Inversion, Slerp).

## 7. Critical Pitfalls for AI
- **IBM Mismatch**: Changing `gs_joint` without updating `gs_model->ibms` will cause mesh melting.
- **Sparse Key Loss**: In rebinding, parents with keys will "drag" children that lack keys. Always perform a Full Bone Sync (Dense Bake) when rebasing.
- **Mirroring Support**: `mat4_decompose` handles negative determinants (reflections) by flipping the scale sign. AI must use this version to prevent 180-degree "twitching" on mirrored limbs.

## 8. Build Workflow
- **`make clean; make all`**: Mandatory after structural math changes to prevent stale binary state ("Paranoid Mode").
- **Linkage**: Use `g++` to resolve C++ dependencies in the static library.
