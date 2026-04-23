# AI Agent Project Guide: GameSkelIO

This document provides essential technical context for AI agents to understand the architecture, data flow, and operational constraints of the GameSkelIO project.

## 1. Project Mission
High-performance 3D skeletal model and animation transcoding. GameSkelIO serves as a high-fidelity bridge between modern formats (GLB, FBX) and engine-ready legacy formats (IQM, SKM).

## 2. Core Architecture: Self-Describing & Automated
The library avoids forcing data into a single internal "canonical" state, opting instead for a self-describing middleware approach.

- **Native-Format Loaders**: Formats are loaded in their **native orientation and winding order**. A GLB remains Y-Up/CCW, and an IQM remains Z-Up/CW.
- **Self-Describing `gs_model`**: The root container tracks its own `orientation` and `winding` state. This metadata allows processing tools to behave context-sensitively.
- **Target-Standard Writers**: Exporters automatically conform the model to their format's required standard (e.g., Y-Up for FBX, Z-Up for IQM) via transparent, in-place conversion on temporary copies.
- **Zero-Copy Orientation API**: `gsk_convert_orientation` performs math transformations directly within existing vertex and matrix buffers, avoiding costly reallocations.
- **Memory-First Design**: All core logic operates on memory buffers (`void* data, size_t size`). Path-based functions are mere wrappers that handle file I/O.

## 3. Data Hierarchy & Operational Constraints
### Data Blocks
- **Geometry**: Parallel vertex arrays (Positions, Normals, UV0/UV1, Tangents, Colors).
- **Skinning**: Bone influences (4-weight limit per vertex) and Joint Indices.
- **Skeleton**: Topological hierarchy with Local TRS and Inverse Bind Matrices (IBMs).
- **Animations**: Sparse, timestamp-based TRS channels.
- **Baked Motion**: On-demand generation of dense `T3, R4, S3` frame buffers.

### Constraints
- **Topological Order**: Joints in the skeleton must be sorted (Parent index < Child index) for hierarchical calculations. `reorder_skeleton()` handles this.
- **Winding Order**: 
    - **CCW**: Standard for GLB/FBX.
    - **CW**: Standard for IQM/SKM.
- **Math**: Column-Major `gs_mat4`. Rotation channels use `xyzw` quaternions.

## 4. Material & Texture System
GameSkelIO bridges the gap between PBR and Legacy (Phong/Lambert) workflows.

- **PBR Suffix Heuristic**: If format-specific PBR flags are missing, the library scans texture names for suffixes like `_albedo`, `_metallic`, `_roughness` to infer intent.
- **Automated PBR Upgrade**: When exporting legacy formats (IQM/SKM) to GLB, materials are automatically upgraded to Metallic-Roughness (Glossiness inverted to Roughness, Metallic set to 0.0).
- **Embedded Textures**: Raw image binary data (PNG/JPG) is preserved in `gs_texture_buffer` arrays and can be retrieved via the `gsk_get_embedded_texture` API.
- **Path Sanitization**: Directory structures and virtual prefixes (e.g., `embedded://`) are stripped during export to ensure clean, relative URIs.

## 5. Specialized Pipelines
- **FBX Logic**: Enforces a `UnitScaleFactor` of `100.0`. Uses strict Object-Property (OP) connection logic to prevent connections-block corruption in binary exports.
- **GLB Logic**: Enforces 4-byte chunk alignment. Uses magic-byte sniffing for extensionless embedded images.
- **Rebinding (`gsk_rebase_pose`)**: Changes rest poses while preserving animation visual invariance by physically warping vertices and mathematically updating IBMs to cancel out the transformation during skinning.

## 6. Component Map
- **`gameskelio.h`**: Public C API. Source of truth for data structures.
- **`gameskelio.cpp`**: Core bridge logic and memory management.
- **`model.h`**: Internal C++ container logic and sampling helpers.
- **`orientation.cpp`**: Coordinate system transformation math.
- **`math_utils.h`**: Linear algebra primitives (Mat4, Quat, Vec3).
- **`glb_writer.cpp` / `fbx_writer.cpp`**: High-fidelity format-specific exporters.

## 7. Build & Maintenance
- **Makefile**: `make clean; make -j8` produces `libgameskelio.a`, `libgameskelio_x64.dll`, and CLI tools.
- **Versioning**: API versions are tracked in `gameskelio.h`. Ensure DLLs and headers are updated in sync.
- **Golden Revision**: `c21f67b1` is the reference point for FBX writer stability. **Caution**: The FBX Object-Property (OP) hierarchy is extremely fragile; structural deviations in connection logic will likely result in **disconnected skinning links** between joints and meshes, causing animations to become inert (tracks are preserved but fail to drive model deformation).
