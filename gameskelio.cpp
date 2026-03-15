#include "gameskelio.h"
#include "model.h"
#include "iqm_loader.h"
#include "glb_loader.h"
#include "fbx_loader.h"
#include "skp_loader.h"
#include "iqm_writer.h"
#include "glb_writer.h"
#include "fbx_writer.h"
#include <cstdlib>
#include <cstring>
#include <string>

static char* my_strdup(const std::string& s) {
    if (s.empty()) return nullptr;
    char* p = (char*)malloc(s.length() + 1);
    strcpy(p, s.c_str());
    return p;
}

static void copy_anim_channel_to_c(const AnimChannel& src, gs_anim_channel& dst, int num_values_per_key) {
    dst.num_keys = src.times.size();
    if (dst.num_keys > 0) {
        dst.times = (double*)malloc(dst.num_keys * sizeof(double));
        memcpy(dst.times, src.times.data(), dst.num_keys * sizeof(double));
        
        dst.values = (float*)malloc(dst.num_keys * num_values_per_key * sizeof(float));
        memcpy(dst.values, src.values.data(), dst.num_keys * num_values_per_key * sizeof(float));
    } else {
        dst.times = nullptr;
        dst.values = nullptr;
    }
}

static gs_model* model_cpp_to_c(const Model& cpp) {
    gs_model* c = (gs_model*)calloc(1, sizeof(gs_model));
    c->qfusion = cpp.qfusion;
    c->has_bounds = cpp.has_bounds;
    memcpy(c->mins, cpp.mins, sizeof(c->mins));
    memcpy(c->maxs, cpp.maxs, sizeof(c->maxs));
    c->radius = cpp.radius;
    c->xyradius = cpp.xyradius;

    c->num_joints = cpp.joints.size();
    if (c->num_joints > 0) {
        c->joints = (gs_joint*)calloc(c->num_joints, sizeof(gs_joint));
        for (size_t i = 0; i < c->num_joints; ++i) {
            c->joints[i].name = my_strdup(cpp.joints[i].name);
            c->joints[i].parent = cpp.joints[i].parent;
            memcpy(c->joints[i].translate, cpp.joints[i].translate, sizeof(c->joints[i].translate));
            memcpy(c->joints[i].rotate, cpp.joints[i].rotate, sizeof(c->joints[i].rotate));
            memcpy(c->joints[i].scale, cpp.joints[i].scale, sizeof(c->joints[i].scale));
        }
    }

    c->num_meshes = cpp.meshes.size();
    if (c->num_meshes > 0) {
        c->meshes = (gs_mesh*)calloc(c->num_meshes, sizeof(gs_mesh));
        for (size_t i = 0; i < c->num_meshes; ++i) {
            c->meshes[i].name = my_strdup(cpp.meshes[i].name);
            c->meshes[i].material_name = my_strdup(cpp.meshes[i].material_name);
            c->meshes[i].color_map = my_strdup(cpp.meshes[i].color_map);
            c->meshes[i].normal_map = my_strdup(cpp.meshes[i].normal_map);
            c->meshes[i].roughness_map = my_strdup(cpp.meshes[i].roughness_map);
            c->meshes[i].occlusion_map = my_strdup(cpp.meshes[i].occlusion_map);
            c->meshes[i].first_vertex = cpp.meshes[i].first_vertex;
            c->meshes[i].num_vertexes = cpp.meshes[i].num_vertexes;
            c->meshes[i].first_triangle = cpp.meshes[i].first_triangle;
            c->meshes[i].num_triangles = cpp.meshes[i].num_triangles;
        }
    }

    c->num_animations = cpp.animations.size();
    if (c->num_animations > 0) {
        c->animations = (gs_animation*)calloc(c->num_animations, sizeof(gs_animation));
        for (size_t i = 0; i < c->num_animations; ++i) {
            c->animations[i].name = my_strdup(cpp.animations[i].name);
            
            c->animations[i].num_bones = cpp.animations[i].bones.size();
            if (c->animations[i].num_bones > 0) {
                c->animations[i].bones = (gs_bone_anim*)calloc(c->animations[i].num_bones, sizeof(gs_bone_anim));
                for (size_t j = 0; j < c->animations[i].num_bones; ++j) {
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].translation, c->animations[i].bones[j].translation, 3);
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].rotation, c->animations[i].bones[j].rotation, 4);
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].scale, c->animations[i].bones[j].scale, 3);
                }
            }
        }
    }

    c->num_vertices = cpp.positions.size() / 3;
    if (c->num_vertices > 0) {
        c->positions = (float*)malloc(cpp.positions.size() * sizeof(float));
        memcpy(c->positions, cpp.positions.data(), cpp.positions.size() * sizeof(float));
    }
    
    if (!cpp.normals.empty()) {
        c->normals = (float*)malloc(cpp.normals.size() * sizeof(float));
        memcpy(c->normals, cpp.normals.data(), cpp.normals.size() * sizeof(float));
    }
    
    if (!cpp.texcoords.empty()) {
        c->texcoords = (float*)malloc(cpp.texcoords.size() * sizeof(float));
        memcpy(c->texcoords, cpp.texcoords.data(), cpp.texcoords.size() * sizeof(float));
    }

    if (!cpp.joints_0.empty()) {
        c->joints_0 = (uint8_t*)malloc(cpp.joints_0.size() * sizeof(uint8_t));
        memcpy(c->joints_0, cpp.joints_0.data(), cpp.joints_0.size() * sizeof(uint8_t));
    }

    if (!cpp.weights_0.empty()) {
        c->weights_0 = (float*)malloc(cpp.weights_0.size() * sizeof(float));
        memcpy(c->weights_0, cpp.weights_0.data(), cpp.weights_0.size() * sizeof(float));
    }

    c->num_indices = cpp.indices.size();
    if (c->num_indices > 0) {
        c->indices = (uint32_t*)malloc(cpp.indices.size() * sizeof(uint32_t));
        memcpy(c->indices, cpp.indices.data(), cpp.indices.size() * sizeof(uint32_t));
    }

    if (!cpp.world_matrices.empty()) {
        c->world_matrices = (gs_mat4*)malloc(cpp.world_matrices.size() * sizeof(gs_mat4));
        memcpy(c->world_matrices, cpp.world_matrices.data(), cpp.world_matrices.size() * sizeof(gs_mat4));
    }

    if (!cpp.ibms.empty()) {
        c->ibms = (gs_mat4*)malloc(cpp.ibms.size() * sizeof(gs_mat4));
        memcpy(c->ibms, cpp.ibms.data(), cpp.ibms.size() * sizeof(gs_mat4));
    }

    if (!cpp.computed_ibms.empty()) {
        c->computed_ibms = (gs_mat4*)malloc(cpp.computed_ibms.size() * sizeof(gs_mat4));
        memcpy(c->computed_ibms, cpp.computed_ibms.data(), cpp.computed_ibms.size() * sizeof(gs_mat4));
    }

    return c;
}

static void copy_anim_channel_to_cpp(const gs_anim_channel& src, AnimChannel& dst, int num_values_per_key) {
    if (src.num_keys > 0 && src.times && src.values) {
        dst.times.assign(src.times, src.times + src.num_keys);
        dst.values.assign(src.values, src.values + src.num_keys * num_values_per_key);
    }
}

static Model model_c_to_cpp(const gs_model* c) {
    Model cpp;
    if (!c) return cpp;

    cpp.qfusion = c->qfusion;
    cpp.has_bounds = c->has_bounds;
    memcpy(cpp.mins, c->mins, sizeof(cpp.mins));
    memcpy(cpp.maxs, c->maxs, sizeof(cpp.maxs));
    cpp.radius = c->radius;
    cpp.xyradius = c->xyradius;

    if (c->num_joints > 0 && c->joints) {
        cpp.joints.resize(c->num_joints);
        for (uint32_t i = 0; i < c->num_joints; ++i) {
            if (c->joints[i].name) cpp.joints[i].name = c->joints[i].name;
            cpp.joints[i].parent = c->joints[i].parent;
            memcpy(cpp.joints[i].translate, c->joints[i].translate, sizeof(c->joints[i].translate));
            memcpy(cpp.joints[i].rotate, c->joints[i].rotate, sizeof(c->joints[i].rotate));
            memcpy(cpp.joints[i].scale, c->joints[i].scale, sizeof(c->joints[i].scale));
        }
    }

    if (c->num_meshes > 0 && c->meshes) {
        cpp.meshes.resize(c->num_meshes);
        for (uint32_t i = 0; i < c->num_meshes; ++i) {
            if (c->meshes[i].name) cpp.meshes[i].name = c->meshes[i].name;
            if (c->meshes[i].material_name) cpp.meshes[i].material_name = c->meshes[i].material_name;
            if (c->meshes[i].color_map) cpp.meshes[i].color_map = c->meshes[i].color_map;
            if (c->meshes[i].normal_map) cpp.meshes[i].normal_map = c->meshes[i].normal_map;
            if (c->meshes[i].roughness_map) cpp.meshes[i].roughness_map = c->meshes[i].roughness_map;
            if (c->meshes[i].occlusion_map) cpp.meshes[i].occlusion_map = c->meshes[i].occlusion_map;
            cpp.meshes[i].first_vertex = c->meshes[i].first_vertex;
            cpp.meshes[i].num_vertexes = c->meshes[i].num_vertexes;
            cpp.meshes[i].first_triangle = c->meshes[i].first_triangle;
            cpp.meshes[i].num_triangles = c->meshes[i].num_triangles;
        }
    }

    if (c->num_animations > 0 && c->animations) {
        cpp.animations.resize(c->num_animations);
        for (uint32_t i = 0; i < c->num_animations; ++i) {
            if (c->animations[i].name) cpp.animations[i].name = c->animations[i].name;
            
            if (c->animations[i].num_bones > 0 && c->animations[i].bones) {
                cpp.animations[i].bones.resize(c->animations[i].num_bones);
                for (uint32_t j = 0; j < c->animations[i].num_bones; ++j) {
                    copy_anim_channel_to_cpp(c->animations[i].bones[j].translation, cpp.animations[i].bones[j].translation, 3);
                    copy_anim_channel_to_cpp(c->animations[i].bones[j].rotation, cpp.animations[i].bones[j].rotation, 4);
                    copy_anim_channel_to_cpp(c->animations[i].bones[j].scale, cpp.animations[i].bones[j].scale, 3);
                }
            }
        }
    }

    if (c->num_vertices > 0) {
        if (c->positions) cpp.positions.assign(c->positions, c->positions + c->num_vertices * 3);
        if (c->normals) cpp.normals.assign(c->normals, c->normals + c->num_vertices * 3);
        if (c->texcoords) cpp.texcoords.assign(c->texcoords, c->texcoords + c->num_vertices * 2);
        if (c->joints_0) cpp.joints_0.assign(c->joints_0, c->joints_0 + c->num_vertices * 4);
        if (c->weights_0) cpp.weights_0.assign(c->weights_0, c->weights_0 + c->num_vertices * 4);
    }

    if (c->num_indices > 0 && c->indices) {
        cpp.indices.assign(c->indices, c->indices + c->num_indices);
    }

    if (c->num_joints > 0) {
        if (c->world_matrices) {
            cpp.world_matrices.resize(c->num_joints);
            memcpy(cpp.world_matrices.data(), c->world_matrices, c->num_joints * sizeof(gs_mat4));
        }
        if (c->ibms) {
            cpp.ibms.resize(c->num_joints);
            memcpy(cpp.ibms.data(), c->ibms, c->num_joints * sizeof(gs_mat4));
        }
        if (c->computed_ibms) {
            cpp.computed_ibms.resize(c->num_joints);
            memcpy(cpp.computed_ibms.data(), c->computed_ibms, c->num_joints * sizeof(gs_mat4));
        }
    }

    return cpp;
}

// ---------------------------------------------------------
// Loaders
// ---------------------------------------------------------

extern "C" gs_model* gsk_load_iqm(const char* path) {
    Model cpp;
    if (load_iqm(path, cpp)) {
        return model_cpp_to_c(cpp);
    }
    return nullptr;
}

extern "C" gs_model* gsk_load_glb(const char* path) {
    Model cpp;
    if (load_glb(path, cpp)) {
        return model_cpp_to_c(cpp);
    }
    return nullptr;
}

extern "C" gs_model* gsk_load_fbx(const char* path) {
    Model cpp;
    if (load_fbx(path, cpp)) {
        return model_cpp_to_c(cpp);
    }
    return nullptr;
}

extern "C" gs_model* gsk_load_skm(const char* path) {
    Model cpp;
    if (load_skm(path, cpp)) {
        return model_cpp_to_c(cpp);
    }
    return nullptr;
}

// ---------------------------------------------------------
// Writers
// ---------------------------------------------------------

extern "C" bool gsk_write_iqm(const char* path, const gs_model* model) {
    if (!model) return false;
    Model cpp = model_c_to_cpp(model);
    return write_iqm(cpp, path);
}

extern "C" bool gsk_write_glb(const char* path, const gs_model* model) {
    if (!model) return false;
    Model cpp = model_c_to_cpp(model);
    return write_glb(cpp, path);
}

extern "C" bool gsk_write_fbx(const char* path, const gs_model* model, bool write_base, bool write_anim) {
    if (!model) return false;
    Model cpp = model_c_to_cpp(model);
    return write_fbx(path, cpp, write_base, write_anim, -1);
}

// ---------------------------------------------------------
// Memory Management
// ---------------------------------------------------------

extern "C" void gsk_free_model(gs_model* model) {
    if (!model) return;

    if (model->joints) {
        for (uint32_t i = 0; i < model->num_joints; ++i) {
            free(model->joints[i].name);
        }
        free(model->joints);
    }

    if (model->meshes) {
        for (uint32_t i = 0; i < model->num_meshes; ++i) {
            free(model->meshes[i].name);
            free(model->meshes[i].material_name);
            free(model->meshes[i].color_map);
            free(model->meshes[i].normal_map);
            free(model->meshes[i].roughness_map);
            free(model->meshes[i].occlusion_map);
        }
        free(model->meshes);
    }

    if (model->animations) {
        for (uint32_t i = 0; i < model->num_animations; ++i) {
            free(model->animations[i].name);
            if (model->animations[i].bones) {
                for (uint32_t j = 0; j < model->animations[i].num_bones; ++j) {
                    free(model->animations[i].bones[j].translation.times);
                    free(model->animations[i].bones[j].translation.values);
                    free(model->animations[i].bones[j].rotation.times);
                    free(model->animations[i].bones[j].rotation.values);
                    free(model->animations[i].bones[j].scale.times);
                    free(model->animations[i].bones[j].scale.values);
                }
                free(model->animations[i].bones);
            }
        }
        free(model->animations);
    }

    free(model->positions);
    free(model->normals);
    free(model->texcoords);
    free(model->joints_0);
    free(model->weights_0);
    free(model->indices);
    free(model->world_matrices);
    free(model->ibms);
    free(model->computed_ibms);

    free(model);
}

// ---------------------------------------------------------
// Operations
// ---------------------------------------------------------

extern "C" void gsk_compute_bind_pose(gs_model* model) {
    if (!model) return;
    Model cpp = model_c_to_cpp(model);
    cpp.compute_bind_pose();
    
    // Copy back the updated matrices
    if (cpp.world_matrices.size() > 0) {
        if (model->world_matrices) free(model->world_matrices);
        model->world_matrices = (gs_mat4*)malloc(cpp.world_matrices.size() * sizeof(gs_mat4));
        memcpy(model->world_matrices, cpp.world_matrices.data(), cpp.world_matrices.size() * sizeof(gs_mat4));
    }
    if (cpp.computed_ibms.size() > 0) {
        if (model->computed_ibms) free(model->computed_ibms);
        model->computed_ibms = (gs_mat4*)malloc(cpp.computed_ibms.size() * sizeof(gs_mat4));
        memcpy(model->computed_ibms, cpp.computed_ibms.data(), cpp.computed_ibms.size() * sizeof(gs_mat4));
    }
}

extern "C" void gsk_compute_bounds(gs_model* model) {
    if (!model) return;
    Model cpp = model_c_to_cpp(model);
    cpp.compute_bounds();
    
    // Copy back
    model->has_bounds = cpp.has_bounds;
    memcpy(model->mins, cpp.mins, sizeof(model->mins));
    memcpy(model->maxs, cpp.maxs, sizeof(model->maxs));
    model->radius = cpp.radius;
    model->xyradius = cpp.xyradius;
}

extern "C" bool gsk_validate_skeleton(gs_model* model) {
    if (!model) return false;
    Model cpp = model_c_to_cpp(model);
    return cpp.validate_skeleton();
}

extern "C" void gsk_reorder_skeleton(gs_model* model) {
    if (!model) return;
    Model cpp = model_c_to_cpp(model);
    cpp.reorder_skeleton();
    
    gs_model* new_c = model_cpp_to_c(cpp);
    
    gs_model temp = *model;
    *model = *new_c;
    *new_c = temp;
    gsk_free_model(new_c);
}
