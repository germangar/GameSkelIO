# AI Agent Project Guide: GameSkelIO

This document is specifically for AI agents to understand the technical nuances, pitfalls, and architectural decisions of the GameSkelIO project.

## 1. Core Technical Context
- **Intermediate Representation (IR)**: Everything centers on the `Model` class in `model.h`.
- **Coordinate System**: Native **Y-Up (Right-Handed)**.
- **Winding Order**: **Counter-Clockwise (CCW)**.
- **Animation Architecture**: Sparse, timestamp-based keyframes (seconds). Do NOT bake frames unless explicitly converting to a format that requires it (like IQM, which handles its own baking/quantization in the writer).
- **Rotation Handling**: Critical. Use the "Continuous Exact Alias Solver" in `math_utils.h` to avoid 180-degree flipping during Euler/Quaternion conversions.

## 2. Component Responsibilities
- **`model.h`**: The Single Source of Truth for data structures. joints use local-space transforms relative to parents.
- **`math_utils.h`**: Contains all transformation logic. If you are changing how a model is rotated or scaled, change it here, not in the loader/writer.
- **`fbx_writer.cpp` / `fbx_loader.cpp`**: High complexity. Uses `ufbx`. Note that FBX often has "Pre-Rotations" and "Post-Rotations" that must be flattened into the `Model` joints.
- **`iqm_loader.cpp` / `iqm_writer.cpp`**: Handles Z-Up (IQM) to Y-Up (Internal) conversion. IQM is "Backward" (CW) winding, so indices are flipped during load/write.
- **`glb_loader.cpp` / `glb_writer.cpp`**: Uses `cgltf`. This is the "reference" format for Y-up coordination; if a model looks right in GLB but wrong in others, the issue is likely in the other format's loader/writer.

## 3. Critical Known Issues & Pitfalls
- **Animation Loop Flickering**: Historical bug fixed by using time-based sampling and specialized edge-case handling at $t=0$ and $t=Duration$.
- **FBX "Arm" Bug**: A specific regression where joints lose scale over time.
  - **GOLDEN COMMIT**: `7e2a6ff` (Free of FBX bugs).
  - **ARM BUG INTRODUCED**: `32126a0`.
- **IQM Animation Merging**: When an `animation.cfg` is present, it must supplement, not overwrite, internal IQM sequences unless the frame range matches exactly.

## 4. Development Workflow for Agents
- **Build**: Use `make -j8` in the root.
- **Cleaning**: Always `make clean` before a full rebuild to ensure `obj/` files are fresh.
- **Verification**:
  - Perform round-trips: `GLB -> IQM -> FBX -> GLB`.
  - Compare the final GLB with the initial one in a viewer (like Three.js or Babylon.js).
- **Git State**: The USER frequently works in "detached HEAD" states to bisect animation bugs. Check `git status` before assuming you are on `main`.

## 5. Implementation Rules
1. **Never use Assimp**: It has been stripped from the project. Use `ufbx` for FBX and `cgltf` for GLB.
2. **Standardize Transforms**: Loaders MUST convert to Y-up/CCW immediately. Writers MUST convert from Y-up/CCW to destination specs.
3. **Euler Aliasing**: When exporting to FBX curves, always use the alias solver to ensure the "shortest path" in rotation space is preserved.
