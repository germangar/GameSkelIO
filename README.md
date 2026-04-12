# GameSkelIO

GameSkelIO is a high-performance C-compatible library designed for **3D skeletal model and animation transcoding**. It serves as a middleware layer between modern 3D formats (like glTF and FBX) and legacy or specialized game engine formats (like IQM and SKM).

The library is currently at **API Version 4**.

## Core Architecture
GameSkelIO has evolved from a single-standard library to a flexible, automated transcoding engine.

- **Self-Describing Models**: The `gs_model` struct contains `orientation` and `winding` fields. Loaders populate these fields so every model is self-aware of its native coordinate system.
- **Embedded Texture Support**: The root `gs_model` struct includes a `textures` array containing raw binary image data (PNG, JPG) extracted from GLB or FBX containers.
- **"Liberated" Loaders**: Loaders (IQM, SKM, GLB, FBX) read data in its native format without forcing a conversion to a single internal standard.
- **"Automated" Writers**: Writers automatically request the standard orientation for their target format (e.g., Y-Up for GLB, Z-Up for IQM).
- **Transparent C API**: The public C functions work on temporary C++ copies, ensuring that the user's original `gs_model*` is never mutated by a write operation.

## Key Features

- **Memory-First API**: Load and export models directly from/to memory buffers.
- **Embedded Texture Retrieval**: Access raw image bytes stored within model files via the `gsk_get_embedded_texture` API.
- **Advanced Orientation API**: Perform in-place orientation and winding order swaps on loaded models.
- **On-Demand Animation Baking**: Convert sparse, time-based animation tracks into dense, frame-based buffers for engines that require it.
- **Format Support**:
  - **IQM**: Full Read/Write (Automates conversion to its Z-Up, CW standard).
  - **GLB/glTF**: Full Read/Write (Strictly writes PBR Metallic-Roughness for maximum compatibility).
  - **FBX (Binary)**: Full Read/Write (Detects native PBR shaders and orientation axes).
  - **SKM/SKP**: Read-only support (Automatically upgraded to PBR materials when exported to GLB).

---

## Building the Project

The project uses a standard Makefile. Running `make` in the root directory will produce:
1. `libgameskelio.a`: The static library for integration.
2. `gskelconv.exe`: A reference command-line tool.

```powershell
make clean
make -j8
```

*Note: Linking requires a C++ linker (e.g., g++) to resolve internal dependencies.*

---

## Library Usage (C Examples)

### 1. Loading and Saving to Disk (Basic)
This example shows a simple transcoding operation from a GLB file to an IQM file. The IQM writer will automatically handle the conversion from GLB's Y-Up space to IQM's Z-Up space.

```c
#include "gameskelio.h"
#include <stdio.h>

// 1. Load a model from disk
gs_model* model = gsk_load_glb("player_input.glb");

if (model) {
    printf("Model loaded with orientation: %d\n", model->orientation);

    // 2. Save the model. The writer automatically handles the conversion.
    if (gsk_write_iqm("player_output.iqm", model, false)) {
        printf("IQM saved successfully.\n");
    }

    // 3. Cleanup
    gsk_free_model(model);
}
```

### 2. Retrieving Embedded Textures
GLB and FBX files often contain embedded textures. You can retrieve the raw binary data (e.g., PNG/JPG bytes) directly from the loaded model.

```c
#include "gameskelio.h"
#include <stdio.h>

gs_model* model = gsk_load_glb("character_with_textures.glb");

if (model) {
    // Get the color map path from the first material
    const char* tex_path = model->materials[0].color_map;
    
    size_t data_size = 0;
    const void* image_bytes = gsk_get_embedded_texture(model, tex_path, &data_size);

    if (image_bytes) {
        printf("Found embedded texture: %s (%zu bytes)\n", tex_path, data_size);
        // You can now pass image_bytes to stbi_load_from_memory() or your engine's loader
    }

    gsk_free_model(model);
}
```

### 3. Advanced Feature: Manual Orientation Control
You can manually convert a model's orientation in-place after loading it.

```c
#include "gameskelio.h"

gs_model* model = gsk_load_fbx("character.fbx");

if (model) {
    printf("Original orientation: %d\n", model->orientation);

    // Manually convert the model to Blender's standard Z-Up coordinate system
    gsk_convert_orientation(model, GS_Z_UP_RIGHTHANDED, GS_WINDING_CW);

    printf("New orientation: %d\n", model->orientation);
    
    gsk_free_model(model);
}
```

### 4. Advanced Feature: Baking Animations
For engines that use frame-based animation systems, you can "bake" any sparse animation into a dense buffer of frame data.

```c
#include "gameskelio.h"

gs_model* model = gsk_load_glb("animated_asset.glb");

if (model && model->num_animations > 0) {
    // Bake the first animation clip at 30 FPS
    uint32_t anim_idx = 0;
    float fps = 30.0f;
    gs_baked_anim* baked_anim = gsk_bake_animation(model, anim_idx, fps);

    if (baked_anim) {
        printf("Animation baked successfully!\n");
        printf("Frames: %u, Joints: %u, FPS: %.1f\n", baked_anim->num_frames, baked_anim->num_joints, baked_anim->fps);

        // IMPORTANT: The baked animation is a new allocation and must be freed
        gsk_free_baked_anim(baked_anim);
    }

    gsk_free_model(model);
}
```

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

## The Rebinding Tool (`gsrebind`)

`gsrebind` is a specialized utility used to change a character's **Bind Pose** (Rest Pose) while ensuring all existing animations remain visually invariant. 

### Usage
```bash
gsrebind <input.glb/iqm/fbx> <anim_idx_or_name> <output.glb/iqm>
```

---

## License
MIT License. See `LICENSE` for details.
