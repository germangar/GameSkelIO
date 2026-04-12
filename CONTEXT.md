# AI Agent Project Guide: GameSkelIO

This document is technical context for AI agents to understand the architecture, data flow, and conventions of the GameSkelIO project.

## 1. Project Overview
- **Type**: C Library (`libgameskelio.a`) with pure C CLI tools.
- **Purpose**: High-performance 3D skeletal model and animation transcoding.
- **Architecture**: **Memory-First**. Loaders and writers operate primarily on buffers (`void* data, size_t size`).
- **Version**: **API Version 4**.

## 2. Core Technical Context & Architecture
The library uses a **Self-Describing & Automated** architecture.

- **Liberated Loaders**: Loaders (FBX, GLB, IQM, SKM) load model data in its **native orientation and winding order**. They no longer force a conversion to a single internal standard.
- **Self-Describing Models**: The `gs_model` struct contains `orientation` and `winding` fields, making every loaded model self-aware of its format.
- **Automated Writers**: Writers (FBX, GLB, IQM) automatically request their standard orientation. This is handled via a **transparent, in-place conversion** on a local copy of the model, ensuring the user's original `gs_model` is never mutated.
- **In-Place Orientation API**: `gsk_convert_orientation` performs all math operations inside existing memory buffers without reallocating any of the model's core pointers.
- **On-Demand Baking API**: `gsk_bake_animation` converts sparse, time-based animation tracks into a dense, frame-based buffer (`T3, R4, S3` floats per joint per frame).
- **Embedded Texture Support**: `gs_model` root struct includes a `textures` array (`gs_texture_buffer`). These store raw binary image data (PNG, JPG) found within GLB or FBX containers. The API call `gsk_get_embedded_texture` allows retrieving this data via string matching on the original path.

## 3. Material & Texture System
The library supports both modern PBR and Legacy (Phong/Lambert) material workflows.

- **Material Detection**: 
    - **GLB/FBX**: Natively identify PBR via format-specific flags. Fall back to a **Suffix-Based Heuristic** (`_albedo`, `_metallic`, `_roughness`, etc.) in texture map names if flags are missing.
    - **IQM/SKM**: Strictly identified as **LEGACY**. 
- **Internal Mapping**: Materials (`gs_material`) track separate maps for PBR (Metallic, Roughness, BaseColor) and Legacy (Specular, Shininess, Diffuse).
- **Automated GLB Conversion**: The GLB writer automatically "converts" Legacy materials into compliant PBR Metallic-Roughness materials by inverting glossiness into roughness and setting a metallic factor of 0.0.
- **Virtual Path Sanitization**: Internal "virtual" paths (e.g., `embedded://texture.png`) are automatically stripped of prefixes and directory paths during export to ensure GLB/FBX files contain clean, relative URIs.

## 4. Data Hierarchy (Public C API)
- **`gs_model`**: The root container. Includes `orientation`, `winding`, and `textures`.
- **`gs_material`**: Property container. Includes `material_type` (PBR=0, Legacy=1).
- **`gs_texture_buffer`**: Raw binary buffer for an image (data, size, original_path).
- **`gs_animation`**: Flattened clip. Contains `gs_bone_anim* bones`.
- **`gs_bone_anim`**: Group of sparse channels (`translation`, `rotation`, `scale`).
- **`gs_baked_anim`**: Standalone, dense frame buffer.

## 5. Format-Specific Logic
- **FBX Writer**:
    - **Standard**: Automates conversion to `Y-Up, CCW`.
    - **Scaling**: Enforces a `UnitScaleFactor` of `100.0`.
    - **Stability**: Strict Object-Property (`OP`) connection logic ensures textures are linked to materials exactly once per material to prevent connections-block corruption.
- **GLB Writer**:
    - **Compliance**: Enforces 4-byte alignment for all binary chunks (`append_to_buffer_aligned`).
    - **Mime-Types**: Uses "Magic Byte" sniffing to identify image types (PNG/JPG) for embedded data lacking extensions.
    - **Strict PBR**: Strictly writes PBR Metallic-Roughness for maximum compatibility with modern viewers (Windows 3D Viewer).
- **IQM/SKM Loaders**:
    - **Legacy**: Always load as `Z-Up, CW` (+X Forward) with Legacy materials.
- **FBX Loader**: Dynamically detects coordinate systems and PBR shaders (Arnold, OSL, Standard Surface) from file metadata.

## 6. Rebinding Architecture (`gsk_rebase_pose`)
Changes rest poses without breaking animations:
- **Mesh Warping**: Physically moves vertices to a new pose.
- **IBM Update**: Updates Inverse Bind Matrices to match the new pose.
- **Mathematical Cancellation**: The warped vertices and updated IBMs mathematically cancel out during skinning, allowing the **Original Local TRS Keys** to be preserved 1:1.

## 7. Component Responsibilities
- **`gameskelio.h`**: Public C API. Reference for all shared data structures.
- **`gameskelio.cpp`**: C++/C bridge. Handles memory management and conversion logic.
- **`model.h`**: Internal C++ Source of Truth. Includes the `has_pbr_suffixes` helper.
- **`glb_writer.cpp` / `fbx_writer.cpp`**: Format-specific exporters with native data embedding.

## 8. Build Workflow
- **`make clean; make -j8`**: Mandatory after structural changes.
- **`libgameskelio_x64.dll`**: The runtime library must be kept in sync with `gameskelio.h` version macros.

## 9. Project History
- **GOLDEN Revision**: `c21f67b1` was the base for the stable FBX writer.
- **Current Main**: Fully restored and enhanced with PBR support, embedded texture workflows, and Windows 3D Viewer compatibility fixes.
