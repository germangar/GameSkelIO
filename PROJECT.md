# AI Agent Project Guide: GameSkelIO

This document is specifically for AI agents to understand the technical nuances, pitfalls, architectural decisions, and the library structure of the GameSkelIO project.

## 1. Project Evolution: C Library
- **Primary Goal**: This project is a **C Library** (`libgameskelio.a`) for 3D model and animation conversion.
- **Public API**: Use `gameskelio.h`. It defines `extern "C"` functions and C-compatible structs (`gs_model`, `gs_joint`, `gs_mesh`, etc.) to allow integration with C, C++, and other languages (FFI).
- **Internal Representation (IR)**: Internally, the library still uses the C++ `Model` class in `model.h` for its loaders and writers. The `gameskelio.cpp` wrapper handles the translation between the C `gs_model` and the internal C++ `Model`.

## 2. Core Technical Context
- **Coordinate System**: Native **Y-Up (Right-Handed)**.
- **Winding Order**: **Counter-Clockwise (CCW)**.
- **Animation Architecture**: Sparse, timestamp-based keyframes (seconds).
- **Frame-Agnostic IR**: The internal `Model` and the public C API are **frame-agnostic**. They use `double* times` and `float* values` in `gs_anim_channel`.
- **Local Conversion**: All frame-to-time (and time-to-frame) calculations are handled locally inside specific loaders (IQM, SKM) and writers (IQM). The internal model never sees or stores `fps`, `first_frame`, or `last_frame`.
- **Rotation Handling**: Critical. Use the "Continuous Exact Alias Solver" in `math_utils.h` to avoid 180-degree flipping during Euler/Quaternion conversions.

## 3. Data Hierarchy (Public API)
- **`gs_model`**: Root container.
- **`gs_animation`**: Contains a `name` and a `gs_animation_track`.
- **`gs_animation_track`**: Contains an array of `gs_bone_anim` (one per joint).
- **`gs_bone_anim`**: Groups `translation`, `rotation`, and `scale` channels for one joint.
- **`gs_anim_channel`**: The raw sparse track. Contains `num_keys`, `times`, and `values`.

## 4. Component Responsibilities
- **`gameskelio.h`**: The public C interface. Use `gsk_load_*` and `gsk_write_*` functions.
- **`gameskelio.cpp`**: The bridge between the C API and the internal C++ implementation.
- **`model.h`**: The internal C++ Source of Truth for data structures.
- **`math_utils.h`**: Contains all transformation logic.
- **`fbx_writer.cpp` / `fbx_loader.cpp`**: Uses `ufbx`. Note that FBX often has "Pre-Rotations" and "Post-Rotations" that must be flattened.
- **`iqm_loader.cpp` / `iqm_writer.cpp`**: Handles Z-Up (IQM) to Y-Up (Internal) conversion. IQM uses CW winding, so indices are flipped.
- **`glb_loader.cpp` / `glb_writer.cpp`**: Uses `cgltf`. Reference format for Y-up coordination.
- **`main.c`**: A pure C command-line tool (`gskelconv.exe`) that serves as a reference implementation and testing utility for the library.

## 5. Critical Known Issues & Pitfalls
- **Memory Management**: When using the C API, you **MUST** call `gsk_free_model(gs_model*)` to release deep allocations. The library uses `malloc/calloc` for C structs.
- **Coordinate Regressions**: When editing loaders, ensure the specific X/Y/Z swaps for root bones and vertices match the committed standard (see `git show HEAD:iqm_loader.cpp`).
- **Standardized Baking**: The IQM writer standardizes on `BASE_FPS` (30.0) when baking sparse tracks back into dense frames for the IQM format.

## 6. Build System
- **`make`**: Builds both `libgameskelio.a` and `gskelconv.exe` in the root.
- **`main.c`** is compiled with `gcc` but linked with `g++` to support C++ internals.
