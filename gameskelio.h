/**
 * GAMESKELIO.H
 * 
 * A C-compatible library for 3D skeletal model and animation conversion.
 * 
 * CORE CONVENTIONS:
 * - Coordinate System: Y-Up, Right-Handed.
 * - Winding Order: Counter-Clockwise (CCW).
 * - Animation: Sparse, timestamp-based (seconds).
 * - Rotations: Quaternions (xyzw).
 */

#ifndef GAMESKELIO_H
#define GAMESKELIO_H

#define GAMESKELIO_VERSION 3

#include <stdint.h>
#include <stdbool.h>

typedef enum gs_coord_system {
    GS_Y_UP_RIGHTHANDED = 0, // glTF, Maya, OpenGL (Default)
    GS_Y_UP_LEFTHANDED,      // Unity, DirectX
    GS_Z_UP_RIGHTHANDED,     // Blender, 3ds Max
    GS_Z_UP_LEFTHANDED,      // Unreal Engine
    GS_X_UP_RIGHTHANDED,     // Rare
    GS_X_UP_LEFTHANDED       // Rare
} gs_coord_system;

typedef enum gs_winding_order {
    GS_WINDING_CCW = 0, // Counter-Clockwise (Default for GLB/FBX)
    GS_WINDING_CW  = 1  // Clockwise (Default for IQM/SKM)
} gs_winding_order;

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * A 4x4 matrix stored as an array of 16 floats.
 * Layout matches standard math library expectations (Column-Major).
 */
typedef struct gs_mat4 {
    float m[16];
} gs_mat4;

/** 
 * A single skeletal joint (bone).
 * Hierarchy is defined by the 'parent' index (referencing the joints array in gs_model).
 * A parent of -1 indicates a root joint.
 */
typedef struct gs_joint {
    char* name;
    int parent;
    float translate[3];
    float rotate[4]; // Quaternion: xyzw
    float scale[3];
} gs_joint;

/** 
 * Defines a mesh segment and its material properties.
 * Refers to a range of vertices and triangles within the global buffers in gs_model.
 */
typedef struct gs_mesh {
    char* name;
    int material_idx;
    uint32_t first_vertex;
    uint32_t num_vertexes;
    uint32_t first_triangle;
    uint32_t num_triangles;
} gs_mesh;

/**
 * Defines material properties for a mesh.
 */
typedef struct gs_material {
    char* name;
    int material_type; // 0 for PBR, 1 for Legacy

    char* color_map;
    char* normal_map;
    char* metallic_map;
    char* roughness_map;
    char* specular_map;
    char* shininess_map;
    char* emissive_map;
    char* occlusion_map;
    char* opacity_map;

    float base_color[4];
    float specular_color[3];
    float emissive_color[3];
    float metallic_factor;
    float roughness_factor;
    float emissive_factor;
} gs_material;

/** 
 * A sparse animation channel for a single property (Translation, Rotation, or Scale).
 * 'times' contains 'num_keys' timestamps in seconds.
 * 'values' contains interleaved data: 3 floats per key for T/S, 4 for R.
 */
typedef struct gs_anim_channel {
    uint32_t num_keys;
    double* times;
    float* values;
} gs_anim_channel;

/** 
 * Groups animation channels for a specific joint.
 */
typedef struct gs_bone_anim {
    gs_anim_channel translation;
    gs_anim_channel rotation;
    gs_anim_channel scale;
} gs_bone_anim;

/**
 * Defines a group of frames for legacy animation formats (like IQM/SKM).
 * Used to override internal animation definitions via external config files.
 */
typedef struct gs_legacy_framegroup {
    const char* name;
    int first_frame;
    int num_frames;
    float fps;
} gs_legacy_framegroup;

/** 
 * A named animation clip (e.g., "Run", "Jump").
 * 'bones' is an array of size 'num_bones', matching the model's num_joints.
 */
typedef struct gs_animation {
    char* name;
    double duration;
    uint32_t num_bones;
    gs_bone_anim* bones;
    uint32_t num_morph_targets;
    gs_anim_channel* morph_weights; // One channel per morph target
} gs_animation;

/**
 * Defines a morph target (blend shape) for a model.
 * Contains vertex attribute deltas.
 */
typedef struct gs_morph_target {
    char* name;
    float* positions; // 3 floats per vertex (xyz deltas)
    float* normals;   // 3 floats per vertex (xyz deltas)
} gs_morph_target;

/** 
 * The root container for a 3D model.
 * Vertex attributes (positions, normals, etc.) are stored in separate parallel arrays.
 * joints_0: 4 joint indices per vertex.
 * weights_0: 4 joint weights per vertex.
 */
/**
 * Defines a baked, frame-based animation.
 * These are generated on-demand from the time-based sparse tracks in gs_model.
 * They are NOT stored as part of the gs_model struct and must be freed manually.
 */
typedef struct gs_baked_anim {
    uint32_t num_frames;
    uint32_t num_joints;
    float fps;
    float* data; // Interleaved data: num_frames * num_joints * 10 (T3, R4, S3)
} gs_baked_anim;

typedef struct gs_model {
    gs_coord_system orientation;
    gs_winding_order winding;

    uint32_t num_joints;
    gs_joint* joints;

    uint32_t num_materials;
    gs_material* materials;

    uint32_t num_meshes;
    gs_mesh* meshes;

    uint32_t num_animations;
    gs_animation* animations;

    uint32_t num_vertices;
    float* positions; // 3 floats per vertex (xyz)
    float* normals;   // 3 floats per vertex (xyz)
    float* texcoords; // 2 floats per vertex (uv)
    float* tangents;   // 4 floats per vertex (xyzw)
    float* colors;     // 4 floats per vertex (rgba)
    float* texcoords_1; // 2 floats per vertex (uv)
    uint8_t* joints_0; // 4 indices per vertex
    float* weights_0; // 4 weights per vertex

    uint32_t num_morph_targets;
    gs_morph_target* morph_targets;

    uint32_t num_indices;
    uint32_t* indices; // 3 indices per triangle

    // Bind-pose transform cache
    gs_mat4* world_matrices;
    gs_mat4* ibms;           // Source Inverse Bind Matrices
    gs_mat4* computed_ibms;  // IBMs computed from local bind pose

    // Metrics
    float mins[3];
    float maxs[3];
    float radius;
    float xyradius;
    bool has_bounds;
} gs_model;

// --- Endpoints ---

// Loaders (Buffer-based): Load model from a memory block.
gs_model* gsk_load_iqm_buffer(const void* data, size_t size, const gs_legacy_framegroup* anims, uint32_t num_anims);
gs_model* gsk_load_glb_buffer(const void* data, size_t size);
gs_model* gsk_load_fbx_buffer(const void* data, size_t size);
gs_model* gsk_load_skm_buffer(const void* skm_data, size_t skm_size, const void* skp_data, size_t skp_size, const gs_legacy_framegroup* anims, uint32_t num_anims);

// Loaders (Path-based): Helper functions that read a file into memory and call the buffer loaders.
gs_model* gsk_load_iqm(const char* path);
gs_model* gsk_load_glb(const char* path);
gs_model* gsk_load_fbx(const char* path);
gs_model* gsk_load_skm(const char* path);

// Writers/Exporters (Buffer-based): Bake model into a specific format's binary representation.
// Returns a pointer to the allocated memory block and sets 'out_size'.
// Caller must release this memory using gsk_free_buffer().
// IQM specific: if 'force_single_anim' is true, all animations are merged into one.
// Optional: provide 'out_anims' and 'out_anim_count' to receive calculated frame metadata.
void* gsk_export_iqm_buffer(const gs_model* model, size_t* out_size, bool force_single_anim, gs_legacy_framegroup** out_anims, uint32_t* out_anim_count);
void* gsk_export_glb_buffer(const gs_model* model, size_t* out_size);

// Writers (Path-based): Helper functions that bake to memory and then write to a file.
bool gsk_write_iqm(const char* path, const gs_model* model, bool force_single_anim);
bool gsk_write_glb(const char* path, const gs_model* model);
bool gsk_write_fbx(const char* path, const gs_model* model, bool write_base, bool write_anim);

// Memory Management
void gsk_free_model(gs_model* model);
void gsk_free_buffer(void* buffer);
void gsk_free_iqm_metadata(gs_legacy_framegroup* anims, uint32_t count);

// Operations: Manipulation and analysis helpers.
uint32_t gsk_get_version(void);
bool gsk_rebase_pose(gs_model* model, uint32_t pose_anim_idx);
bool gsk_move_animation(gs_model* model, uint32_t from_idx, uint32_t to_idx);
void gsk_compute_bind_pose(gs_model* model);
void gsk_compute_bounds(gs_model* model);
bool gsk_validate_skeleton(gs_model* model);
bool gsk_reorder_skeleton(gs_model* model);
bool gsk_convert_orientation(gs_model* model, gs_coord_system target_orientation, gs_winding_order target_winding);

/**
 * Bakes a specific animation clip into a fixed-rate frame buffer.
 * The resulting gs_baked_anim is allocated on-demand and must be freed with gsk_free_baked_anim.
 * It is NOT part of the gs_model lifetime.
 */
gs_baked_anim* gsk_bake_animation(const gs_model* model, uint32_t anim_idx, float fps);
void gsk_free_baked_anim(gs_baked_anim* baked);

#ifdef __cplusplus
}
#endif

#endif // GAMESKELIO_H
