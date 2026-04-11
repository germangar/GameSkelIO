# AI Agent Project Guide: GameSkelIO

This document is technical context for AI agents to understand the architecture, data flow, and conventions of the GameSkelIO project.

## 1. Project Overview
- **Type**: C Library (`libgameskelio.a`) with pure C CLI tools.
- **Purpose**: High-performance 3D skeletal model and animation transcoding.
- **Architecture**: **Memory-First**. Loaders and writers operate primarily on buffers (`void* data, size_t size`).

## 2. Core Technical Context & Architecture
The library has shifted from a single core convention to a **Self-Describing & Automated** architecture.

- **Liberated Loaders**: Loaders (FBX, GLB, IQM, SKM) now load model data in its **native orientation and winding order**. They no longer force a conversion to a single internal standard.
- **Self-Describing Models**: The `gs_model` struct now contains `gs_coord_system orientation` and `gs_winding_order winding` fields, making every loaded model self-aware of its format.
- **Automated Writers**: Writers (FBX, GLB, IQM) automatically request their standard orientation. This is handled via a **transparent, in-place conversion** on a local copy of the model, ensuring the user's original `gs_model` is never mutated.
- **In-Place Orientation API**: `gsk_convert_orientation` performs all math operations inside existing memory buffers without reallocating any of the model's core pointers.
- **On-Demand Baking API**: `gsk_bake_animation` converts sparse, time-based animation tracks into a dense, frame-based buffer (`T3, R4, S3` floats per joint per frame) upon request. This baked data is not part of the standard `gs_model` lifecycle.
- **Mathematically Sound Bases**: All coordinate system conversions are derived from a set of mathematically consistent basis vectors in `src/orientation.cpp`, where Right-Handed systems have a determinant of `+1` and Left-Handed systems have `-1`.

## 3. Data Hierarchy (Public C API)
- **`gs_model`**: The root container. **Now includes `orientation` and `winding`.**
- **`gs_animation`**: Flattened clip. Contains `num_bones` and `gs_bone_anim* bones`.
- **`gs_bone_anim`**: Group of channels (`translation`, `rotation`, `scale`) for one joint.
- **`gs_anim_channel`**: Raw sparse track. Contains `num_keys`, `double* times`, and `float* values`.
- **`gs_baked_anim`**: **New struct**. Contains the output of `gsk_bake_animation`. It is a standalone, allocated buffer and must be freed with `gsk_free_baked_anim`.
- **`gs_legacy_framegroup`**: Metadata struct for format overrides (name, first_frame, num_frames, fps).

## 4. Format-Specific Logic
- **FBX Writer**:
    - **Standard**: Automates conversion to `Y-Up, CCW`.
    - **Scaling**: Enforces a `UnitScaleFactor` of `100.0` and multiplies all positional data by 100.
    - **Connections**: Preserves the critical `Connections` block at the end of the file for skeleton and mesh linking.
- **IQM/SKM Loaders**:
    - **Native**: Load as `Z-Up, CW` (+X Forward).
    - **Overrides**: Support `gs_legacy_framegroup` overrides from `.cfg` files.
- **IQM Writer**:
    - **Standard**: Automates conversion to `Z-Up, CW`.
    - **Baking**: Uses the `gsk_bake_animation` logic to generate frame data.
- **GLB Loader**: Loads as standard `Y-Up, CCW`.
- **GLB Writer**: Automates conversion to `Y-Up, CCW`.
- **FBX Loader**: Dynamically detects the coordinate system from the file's metadata and sets the model's `orientation` field accordingly.

## 5. Rebinding Architecture (`gsk_rebase_pose`)
The rebind tool uses a **Mathematical Cancellation** strategy to change rest poses without breaking animations:
- **Mesh Warping**: Vertices are physically moved to a new pose (e.g., A-Pose).
- **IBM Update**: Inverse Bind Matrices are updated to match the new pose.
- **Cancellation**: Since $V_{warped} = (W_{new\_bind} \cdot IBM_{old}) \cdot V_{original}$, and $IBM_{new} = W_{new\_bind}^{-1}$, the equation $W_{anim} \cdot IBM_{new} \cdot V_{warped}$ simplifies back to $W_{anim} \cdot IBM_{old} \cdot V_{original}$.
- **Local Track Retention**: Because of this cancellation, the **Original Local TRS Keys** can be preserved 1:1, ensuring bit-perfect world-space motion.

## 6. Component Responsibilities
- **`gameskelio.h`**: The public C API. Main reference for data structures.
- **`gameskelio.cpp`**: C++/C bridge. Contains `gsk_rebase_pose`, `gsk_convert_orientation`, and `gsk_bake_animation`.
- **`model.h`**: Internal C++ "Source of Truth" representation. Contains the core `bake_animation` C++ method.
- **`orientation.cpp`**: The heart of the new orientation system. Contains the basis vectors and matrix math for conversions.
- **`math_utils.h`**: Critical math operations (Decomposition, Inversion, Slerp).

## 7. Critical Pitfalls for AI
- **IBM Mismatch**: Changing `gs_joint` without updating `gs_model->ibms` will cause mesh melting.
- **Dangling Pointers**: When using the local-copy pattern in writers, do not store `const char*` from the temporary copy in metadata that outlives the function. Point to the original `model_in` string data instead.
- **Mirroring Support**: `mat4_decompose` handles negative determinants (reflections) by flipping the scale sign. AI must use this version to prevent 180-degree "twitching" on mirrored limbs.

## 8. Build Workflow
- **`make clean; make -j8`**: Mandatory after structural math changes to prevent stale binary state ("Paranoid Mode").
- **Linkage**: Use `g++` to resolve C++ dependencies in the static library.

## 9. Project History & Known Regressions
- **GOLDEN Revision**: `c21f67b1` is the "Known Good" base for the FBX writer architecture, before the multi-mesh and PBR features were re-applied. The `main` branch now contains a fully restored and enhanced writer based on this commit.
- **Bug Tracking**: The "ROOT ROTATED" and "ARM BUG" issues from previous branches are considered **RESOLVED** on the current `main` branch.
