# GameSkelIO

GameSkelIO is a high-performance C-compatible library designed for **3D skeletal model and animation transcoding**. It serves as a middleware layer between modern 3D formats (like glTF and FBX) and legacy or specialized game engine formats (like IQM and SKM).

The library is designed with a **memory-first architecture**, allowing it to be easily integrated into game engine Virtual File Systems (VFS) without disk I/O overhead.

## Key Features

- **Memory-First API**: Load and export models directly from/to memory buffers.
- **Frame-Agnostic Core**: Internally operates on sparse, timestamp-based animation tracks (seconds), making it strictly format-neutral.
- **Format Support**:
  - **IQM**: Full Read/Write (Z-up to Y-up conversion, CW to CCW winding).
  - **GLB/glTF**: Full Read/Write (Native Y-up reference).
  - **FBX (Binary)**: Full Read/Write (Uses high-performance `ufbx` backend).
  - **SKM/SKP**: Read-only support for legacy Warsow/Warfork formats.
- **Legacy Engine Support**: Specific toggles for forcing single-animation stacks and automated metadata generation for `.cfg` files.
- **Precision Baking**: Linear duration snapping ensures that sparse tracks are baked into dense frames (e.g., 30 FPS) with perfect alignment.

---

## Building the Project

The project uses a standard Makefile. Running `make` in the root directory will produce:
1. `libgameskelio.a`: The static library for integration.
2. `gskelconv.exe`: A reference command-line tool.

```powershell
make clean
make
```

*Note: Linking requires a C++ linker (e.g., g++) to resolve internal dependencies.*

---

## Library Usage (C Examples)

### 1. Loading and Saving to Disk (Model Editor/Viewer)
If you are building a 3D tool like a viewer or an editor, you can load models directly from disk into the `gs_model` representation, manipulate the data, and save it back to any supported format.

```c
#include "gameskelio.h"
#include <stdio.h>

// 1. Load a model from disk
gs_model* model = gsk_load_glb("player_input.glb");

if (model) {
    // 2. Access and manipulate data directly
    printf("Model loaded: %u vertices, %u joints\n", model->num_vertices, model->num_joints);

    // Derived data calculation
    gsk_compute_bind_pose(model);
    gsk_compute_bounds(model);

    // 3. Save the model back to disk in a different format
    // Parameters: path, model, force_single_anim
    if (gsk_write_iqm("player_output.iqm", model, false)) {
        printf("IQM saved successfully.\n");
    }

    // 4. Cleanup
    gsk_free_model(model);
}
```

### 2. In-Memory Transcoding (Engine Integration)
For high-performance engine integration, GameSkelIO can act as a buffer-to-buffer transcoder, allowing you to load assets from your engine's VFS and convert them without touching the disk.

```c
#include "gameskelio.h"

// 1. Load a model from a memory buffer (e.g., from a .pak or .pk3 file)
size_t glb_size = /* size from VFS */;
void* glb_data = /* pointer from VFS */;
gs_model* model = gsk_load_glb_buffer(glb_data, glb_size);

if (model) {
    // 2. Initialize derived data (Bounds, Bind Pose)
    gsk_compute_bind_pose(model);
    gsk_compute_bounds(model);

    // 3. Export to a memory-baked IQM buffer for the engine
    size_t iqm_size = 0;
    bool force_single = true; // Merge all frames into a single animation?

    void* iqm_buffer = gsk_export_iqm_buffer(model, &iqm_size, force_single, NULL, NULL);

    if (iqm_buffer) {
        // Use 'iqm_buffer' with your engine's native IQM loader...

        // 4. Cleanup
        gsk_free_buffer(iqm_buffer);
    }

    gsk_free_model(model);
}
```

```

---

## The Converter Tool (`gskelconv`)

`gskelconv` is a command-line utility included with the library. It demonstrates the library's capabilities and is useful for batch processing.

### Usage
```bash
gskelconv <input.iqm/glb/skm/fbx> <output.iqm/glb/fbx> [flags]
```

### Flags
- `--qfusion`: Repurposes the IQM writer to force a single animation stack (essential for QFusion/Warfork compatibility) and automatically generates a matching `<model>.cfg` file.
- `--base`: (FBX only) Export the base pose only.
- `--anim`: (FBX only) Export animations only.

### Example
```bash
./gskelconv player.glb player.iqm --qfusion
```

---

## Technical Specifications

To ensure consistency across all formats, GameSkelIO standardizes data into the following Intermediate Representation (IR):

- **Coordinate System**: Y-Up, Right-Handed.
- **Winding Order**: Counter-Clockwise (CCW).
- **Rotations**: Quaternions (xyzw).
- **Animations**: Sparse tracks with `double` precision timestamps.
- **Joints**: Hierarchical parent-index system.
- **Weights**: Normalized 4-bone per vertex influence.

---

## License
MIT License. See `LICENSE` for details.
