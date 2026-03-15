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

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * A 4x4 matrix stored as an array of 16 floats.
 * Layout matches standard math library expectations (Row-Major).
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
    char* material_name;
    char* color_map;
    char* normal_map;
    char* roughness_map;
    char* occlusion_map;
    uint32_t first_vertex;
    uint32_t num_vertexes;
    uint32_t first_triangle;
    uint32_t num_triangles;
} gs_mesh;

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
 * A named animation clip (e.g., "Run", "Jump").
 * 'bones' is an array of size 'num_bones', matching the model's num_joints.
 */
typedef struct gs_animation {
    char* name;
    uint32_t num_bones;
    gs_bone_anim* bones;
} gs_animation;

/** 
 * The root container for a 3D model.
 * Vertex attributes (positions, normals, etc.) are stored in separate parallel arrays.
 * joints_0: 4 joint indices per vertex.
 * weights_0: 4 joint weights per vertex.
 */
typedef struct gs_model {
    uint32_t num_joints;
    gs_joint* joints;

    uint32_t num_meshes;
    gs_mesh* meshes;

    uint32_t num_animations;
    gs_animation* animations;

    uint32_t num_vertices;
    float* positions; // 3 floats per vertex (xyz)
    float* normals;   // 3 floats per vertex (xyz)
    float* texcoords; // 2 floats per vertex (uv)
    uint8_t* joints_0; // 4 indices per vertex
    float* weights_0; // 4 weights per vertex

    uint32_t num_indices;
    uint32_t* indices; // 3 indices per triangle

    bool qfusion; // Optimization flag for specific engine backends

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

// Loaders: Load model from file. Returns NULL on failure.
gs_model* gsk_load_iqm(const char* path);
gs_model* gsk_load_glb(const char* path);
gs_model* gsk_load_fbx(const char* path);
gs_model* gsk_load_skm(const char* path);

// Writers: Save model to file. Returns true on success.
bool gsk_write_iqm(const char* path, const gs_model* model);
bool gsk_write_glb(const char* path, const gs_model* model);
bool gsk_write_fbx(const char* path, const gs_model* model, bool write_base, bool write_anim);

// Memory Management: Release all memory allocated for a model.
void gsk_free_model(gs_model* model);

// Operations: Manipulation and analysis helpers.
void gsk_compute_bind_pose(gs_model* model);
void gsk_compute_bounds(gs_model* model);
bool gsk_validate_skeleton(gs_model* model);
void gsk_reorder_skeleton(gs_model* model);

#ifdef __cplusplus
}
#endif

#endif // GAMESKELIO_H
