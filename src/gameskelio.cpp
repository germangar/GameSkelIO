#include "gameskelio.h"
#include "model.h"
#include "iqm_loader.h"
#include "glb_loader.h"
#include "fbx_loader.h"
#include "skp_loader.h"
#include "iqm_writer.h"
#include "glb_writer.h"
#include "fbx_writer.h"
#include "orientation.h"
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <set>

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
    c->orientation = cpp.orientation;
    c->winding = cpp.winding;
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

    c->num_materials = cpp.materials.size();
    if (c->num_materials > 0) {
        c->materials = (gs_material*)calloc(c->num_materials, sizeof(gs_material));
        for (size_t i = 0; i < c->num_materials; ++i) {
            const Material& src = cpp.materials[i];
            gs_material& dst = c->materials[i];
            dst.name = my_strdup(src.name);
            dst.material_type = src.material_type;
            dst.color_map = my_strdup(src.color_map);
            dst.normal_map = my_strdup(src.normal_map);
            dst.metallic_map = my_strdup(src.metallic_map);
            dst.roughness_map = my_strdup(src.roughness_map);
            dst.specular_map = my_strdup(src.specular_map);
            dst.shininess_map = my_strdup(src.shininess_map);
            dst.emissive_map = my_strdup(src.emissive_map);
            dst.occlusion_map = my_strdup(src.occlusion_map);
            dst.opacity_map = my_strdup(src.opacity_map);
            memcpy(dst.base_color, src.base_color, sizeof(dst.base_color));
            memcpy(dst.specular_color, src.specular_color, sizeof(dst.specular_color));
            memcpy(dst.emissive_color, src.emissive_color, sizeof(dst.emissive_color));
            dst.metallic_factor = src.metallic_factor;
            dst.roughness_factor = src.roughness_factor;
            dst.emissive_factor = src.emissive_factor;
        }
    }

    c->num_meshes = cpp.meshes.size();
    if (c->num_meshes > 0) {
        c->meshes = (gs_mesh*)calloc(c->num_meshes, sizeof(gs_mesh));
        for (size_t i = 0; i < c->num_meshes; ++i) {
            c->meshes[i].name = my_strdup(cpp.meshes[i].name);
            c->meshes[i].material_idx = cpp.meshes[i].material_idx;
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
            c->animations[i].duration = cpp.animations[i].duration;
            c->animations[i].num_bones = cpp.animations[i].bones.size();
            if (c->animations[i].num_bones > 0) {
                c->animations[i].bones = (gs_bone_anim*)calloc(c->animations[i].num_bones, sizeof(gs_bone_anim));
                for (size_t j = 0; j < c->animations[i].num_bones; ++j) {
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].translation, c->animations[i].bones[j].translation, 3);
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].rotation, c->animations[i].bones[j].rotation, 4);
                    copy_anim_channel_to_c(cpp.animations[i].bones[j].scale, c->animations[i].bones[j].scale, 3);
                }
            }
            c->animations[i].num_morph_targets = cpp.animations[i].morph_weights.size();
            if (c->animations[i].num_morph_targets > 0) {
                c->animations[i].morph_weights = (gs_anim_channel*)calloc(c->animations[i].num_morph_targets, sizeof(gs_anim_channel));
                for (size_t j = 0; j < c->animations[i].num_morph_targets; ++j) {
                    copy_anim_channel_to_c(cpp.animations[i].morph_weights[j], c->animations[i].morph_weights[j], 1);
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

    if (!cpp.tangents.empty()) {
        c->tangents = (float*)malloc(cpp.tangents.size() * sizeof(float));
        memcpy(c->tangents, cpp.tangents.data(), cpp.tangents.size() * sizeof(float));
    }

    if (!cpp.colors.empty()) {
        c->colors = (float*)malloc(cpp.colors.size() * sizeof(float));
        memcpy(c->colors, cpp.colors.data(), cpp.colors.size() * sizeof(float));
    }

    if (!cpp.texcoords_1.empty()) {
        c->texcoords_1 = (float*)malloc(cpp.texcoords_1.size() * sizeof(float));
        memcpy(c->texcoords_1, cpp.texcoords_1.data(), cpp.texcoords_1.size() * sizeof(float));
    }

    if (!cpp.joints_0.empty()) {
        c->joints_0 = (uint8_t*)malloc(cpp.joints_0.size() * sizeof(uint8_t));
        memcpy(c->joints_0, cpp.joints_0.data(), cpp.joints_0.size() * sizeof(uint8_t));
    }

    if (!cpp.weights_0.empty()) {
        c->weights_0 = (float*)malloc(cpp.weights_0.size() * sizeof(float));
        memcpy(c->weights_0, cpp.weights_0.data(), cpp.weights_0.size() * sizeof(float));
    }

    c->num_morph_targets = cpp.morph_targets.size();
    if (c->num_morph_targets > 0) {
        c->morph_targets = (gs_morph_target*)calloc(c->num_morph_targets, sizeof(gs_morph_target));
        for (size_t i = 0; i < c->num_morph_targets; ++i) {
            c->morph_targets[i].name = my_strdup(cpp.morph_targets[i].name);
            if (!cpp.morph_targets[i].positions.empty()) {
                c->morph_targets[i].positions = (float*)malloc(cpp.morph_targets[i].positions.size() * sizeof(float));
                memcpy(c->morph_targets[i].positions, cpp.morph_targets[i].positions.data(), cpp.morph_targets[i].positions.size() * sizeof(float));
            }
            if (!cpp.morph_targets[i].normals.empty()) {
                c->morph_targets[i].normals = (float*)malloc(cpp.morph_targets[i].normals.size() * sizeof(float));
                memcpy(c->morph_targets[i].normals, cpp.morph_targets[i].normals.data(), cpp.morph_targets[i].normals.size() * sizeof(float));
            }
        }
    }

    c->num_indices = cpp.indices.size();
    if (c->num_indices > 0 && !cpp.indices.empty()) {
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

    cpp.orientation = c->orientation;
    cpp.winding = c->winding;
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

    if (c->num_materials > 0 && c->materials) {
        cpp.materials.resize(c->num_materials);
        for (uint32_t i = 0; i < c->num_materials; ++i) {
            const gs_material& src = c->materials[i];
            Material& dst = cpp.materials[i];
            if (src.name) dst.name = src.name;
            dst.material_type = src.material_type;
            if (src.color_map) dst.color_map = src.color_map;
            if (src.normal_map) dst.normal_map = src.normal_map;
            if (src.metallic_map) dst.metallic_map = src.metallic_map;
            if (src.roughness_map) dst.roughness_map = src.roughness_map;
            if (src.specular_map) dst.specular_map = src.specular_map;
            if (src.shininess_map) dst.shininess_map = src.shininess_map;
            if (src.emissive_map) dst.emissive_map = src.emissive_map;
            if (src.occlusion_map) dst.occlusion_map = src.occlusion_map;
            if (src.opacity_map) dst.opacity_map = src.opacity_map;
            memcpy(dst.base_color, src.base_color, sizeof(dst.base_color));
            memcpy(dst.specular_color, src.specular_color, sizeof(dst.specular_color));
            memcpy(dst.emissive_color, src.emissive_color, sizeof(dst.emissive_color));
            dst.metallic_factor = src.metallic_factor;
            dst.roughness_factor = src.roughness_factor;
            dst.emissive_factor = src.emissive_factor;
        }
    }

    if (c->num_meshes > 0 && c->meshes) {
        cpp.meshes.resize(c->num_meshes);
        for (uint32_t i = 0; i < c->num_meshes; ++i) {
            if (c->meshes[i].name) cpp.meshes[i].name = c->meshes[i].name;
            cpp.meshes[i].material_idx = c->meshes[i].material_idx;
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
            if (c->animations[i].num_morph_targets > 0 && c->animations[i].morph_weights) {
                cpp.animations[i].morph_weights.resize(c->animations[i].num_morph_targets);
                for (uint32_t j = 0; j < c->animations[i].num_morph_targets; ++j) {
                    copy_anim_channel_to_cpp(c->animations[i].morph_weights[j], cpp.animations[i].morph_weights[j], 1);
                }
            }
        }
    }

    if (c->num_vertices > 0) {
        if (c->positions) cpp.positions.assign(c->positions, c->positions + c->num_vertices * 3);
        if (c->normals) cpp.normals.assign(c->normals, c->normals + c->num_vertices * 3);
        if (c->texcoords) cpp.texcoords.assign(c->texcoords, c->texcoords + c->num_vertices * 2);
        if (c->tangents) cpp.tangents.assign(c->tangents, c->tangents + c->num_vertices * 4);
        if (c->colors) cpp.colors.assign(c->colors, c->colors + c->num_vertices * 4);
        if (c->texcoords_1) cpp.texcoords_1.assign(c->texcoords_1, c->texcoords_1 + c->num_vertices * 2);
        if (c->joints_0) cpp.joints_0.assign(c->joints_0, c->joints_0 + c->num_vertices * 4);
        if (c->weights_0) cpp.weights_0.assign(c->weights_0, c->weights_0 + c->num_vertices * 4);
    }

    if (c->num_morph_targets > 0 && c->morph_targets) {
        cpp.morph_targets.resize(c->num_morph_targets);
        for (uint32_t i = 0; i < c->num_morph_targets; ++i) {
            if (c->morph_targets[i].name) cpp.morph_targets[i].name = c->morph_targets[i].name;
            if (c->morph_targets[i].positions) {
                cpp.morph_targets[i].positions.assign(c->morph_targets[i].positions, c->morph_targets[i].positions + c->num_vertices * 3);
            }
            if (c->morph_targets[i].normals) {
                cpp.morph_targets[i].normals.assign(c->morph_targets[i].normals, c->morph_targets[i].normals + c->num_vertices * 3);
            }
        }
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

extern "C" gs_model* gsk_load_iqm_buffer(const void* data, size_t size, const gs_legacy_framegroup* anims, uint32_t num_anims) {
    Model cpp;
    if (load_iqm_from_memory(data, size, cpp, anims, num_anims)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_glb_buffer(const void* data, size_t size) {
    Model cpp;
    if (load_glb_from_memory(data, size, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_fbx_buffer(const void* data, size_t size) {
    Model cpp;
    if (load_fbx_from_memory(data, size, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_skm_buffer(const void* skm_data, size_t skm_size, const void* skp_data, size_t skp_size, const gs_legacy_framegroup* anims, uint32_t num_anims) {
    Model cpp;
    if (load_skm_from_memory(skm_data, skm_size, skp_data, skp_size, cpp, anims, num_anims)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_iqm(const char* path) {
    Model cpp;
    if (load_iqm(path, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_glb(const char* path) {
    Model cpp;
    if (load_glb(path, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_fbx(const char* path) {
    Model cpp;
    if (load_fbx(path, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

extern "C" gs_model* gsk_load_skm(const char* path) {
    Model cpp;
    if (load_skm(path, cpp)) return model_cpp_to_c(cpp);
    return nullptr;
}

// ---------------------------------------------------------
// Writers
// ---------------------------------------------------------

extern "C" void* gsk_export_iqm_buffer(const gs_model* model, size_t* out_size, bool force_single_anim, gs_legacy_framegroup** out_anims, uint32_t* out_anim_count) {
    if (!model || !out_size) return nullptr;
    Model cpp = model_c_to_cpp(model);
    
    std::vector<gs_legacy_framegroup> metadata;
    std::vector<uint8_t> buffer = write_iqm_to_memory(cpp, force_single_anim, out_anims ? &metadata : nullptr);
    
    if (buffer.empty()) {
        *out_size = 0;
        return nullptr;
    }

    if (out_anims && out_anim_count) {
        *out_anim_count = (uint32_t)metadata.size();
        *out_anims = (gs_legacy_framegroup*)malloc(metadata.size() * sizeof(gs_legacy_framegroup));
        for (size_t i = 0; i < metadata.size(); ++i) {
            (*out_anims)[i] = metadata[i];
            (*out_anims)[i].name = my_strdup(metadata[i].name);
        }
    }

    *out_size = buffer.size();
    void* ptr = malloc(buffer.size());
    memcpy(ptr, buffer.data(), buffer.size());
    return ptr;
}

extern "C" void gsk_free_iqm_metadata(gs_legacy_framegroup* anims, uint32_t count) {
    if (!anims) return;
    for (uint32_t i = 0; i < count; ++i) {
        if (anims[i].name) free((void*)anims[i].name);
    }
    free(anims);
}

extern "C" void* gsk_export_glb_buffer(const gs_model* model, size_t* out_size) {
    if (!model || !out_size) return nullptr;
    Model cpp = model_c_to_cpp(model);
    std::vector<uint8_t> buffer = write_glb_to_memory(cpp);
    if (buffer.empty()) {
        *out_size = 0;
        return nullptr;
    }
    *out_size = buffer.size();
    void* ptr = malloc(buffer.size());
    memcpy(ptr, buffer.data(), buffer.size());
    return ptr;
}

extern "C" bool gsk_write_iqm(const char* path, const gs_model* model, bool force_single_anim) {
    if (!model) return false;
    Model cpp = model_c_to_cpp(model);
    return write_iqm(cpp, path, force_single_anim);
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
        for (uint32_t i = 0; i < model->num_joints; ++i) free(model->joints[i].name);
        free(model->joints);
    }

    if (model->materials) {
        for (uint32_t i = 0; i < model->num_materials; ++i) {
            free(model->materials[i].name);
            free(model->materials[i].color_map);
            free(model->materials[i].normal_map);
            free(model->materials[i].metallic_map);
            free(model->materials[i].roughness_map);
            free(model->materials[i].specular_map);
            free(model->materials[i].shininess_map);
            free(model->materials[i].emissive_map);
            free(model->materials[i].occlusion_map);
            free(model->materials[i].opacity_map);
        }
        free(model->materials);
    }

    if (model->meshes) {
        for (uint32_t i = 0; i < model->num_meshes; ++i) {
            free(model->meshes[i].name);
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
            if (model->animations[i].morph_weights) {
                for (uint32_t j = 0; j < model->animations[i].num_morph_targets; ++j) {
                    free(model->animations[i].morph_weights[j].times);
                    free(model->animations[i].morph_weights[j].values);
                }
                free(model->animations[i].morph_weights);
            }
        }
        free(model->animations);
    }

    free(model->positions);
    free(model->normals);
    free(model->texcoords);
    free(model->tangents);
    free(model->colors);
    free(model->texcoords_1);
    free(model->joints_0);
    free(model->weights_0);

    if (model->morph_targets) {
        for (uint32_t i = 0; i < model->num_morph_targets; ++i) {
            free(model->morph_targets[i].name);
            free(model->morph_targets[i].positions);
            free(model->morph_targets[i].normals);
        }
        free(model->morph_targets);
    }

    free(model->indices);
    free(model->world_matrices);
    free(model->ibms);
    free(model->computed_ibms);

    free(model);
}

extern "C" void gsk_free_buffer(void* buffer) {
    free(buffer);
}

// ---------------------------------------------------------
// Operations
// ---------------------------------------------------------

extern "C" void gsk_compute_bind_pose(gs_model* model) {
    if (!model) return;
    Model cpp = model_c_to_cpp(model);
    cpp.compute_bind_pose();
    
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
extern "C" uint32_t gsk_get_version(void) {
    return 3;
}

extern "C" bool gsk_move_animation(gs_model* model, uint32_t from_idx, uint32_t to_idx) {
    if (!model || from_idx >= model->num_animations || to_idx >= model->num_animations) return false;
    if (from_idx == to_idx) return true;

    gs_animation temp = model->animations[from_idx];
    if (from_idx < to_idx) {
        memmove(&model->animations[from_idx], &model->animations[from_idx + 1], (to_idx - from_idx) * sizeof(gs_animation));
    } else {
        memmove(&model->animations[to_idx + 1], &model->animations[to_idx], (from_idx - to_idx) * sizeof(gs_animation));
    }
    model->animations[to_idx] = temp;
    return true;
}

extern "C" bool gsk_rebase_pose(gs_model* model, uint32_t pose_anim_idx) {
    if (!model || pose_anim_idx >= model->num_animations) return false;
    
    // Convert to C++ model for math
    Model cpp = model_c_to_cpp(model);
    
    // 0. Compute ORIGINAL World Binds and IBMs
    // Make sure we have a fresh set of matrices for the STARTing state
    cpp.compute_bind_pose();
    std::vector<mat4> old_world_binds = cpp.world_matrices;
    std::vector<mat4> old_ibms = (cpp.ibms.size() == cpp.joints.size()) ? cpp.ibms : cpp.computed_ibms;
    std::vector<mat4> old_locals(cpp.joints.size());
    for(size_t i=0; i<cpp.joints.size(); ++i) {
        old_locals[i] = mat4_from_trs(cpp.joints[i].translate, cpp.joints[i].rotate, cpp.joints[i].scale);
    }

    // 1. Compute NEW World Binds and IBMs
    // To handle non-topological joint orders, we temporarily inject the target pose into the joints list
    std::vector<Pose> target_poses;
    cpp.evaluate_animation(pose_anim_idx, 0.0, target_poses);
    
    std::vector<Joint> saved_rest_pose = cpp.joints;
    for(size_t i=0; i<cpp.joints.size(); ++i) {
        memcpy(cpp.joints[i].translate, target_poses[i].translate, 12);
        memcpy(cpp.joints[i].rotate, target_poses[i].rotate, 16);
        memcpy(cpp.joints[i].scale, target_poses[i].scale, 12);
    }
    cpp.compute_bind_pose(); // Solid iterative pass
    std::vector<mat4> new_world_binds = cpp.world_matrices;
    std::vector<mat4> new_ibms = cpp.computed_ibms;
    std::vector<mat4> new_locals(cpp.joints.size());
    for(size_t i=0; i<cpp.joints.size(); ++i) {
        new_locals[i] = mat4_from_trs(cpp.joints[i].translate, cpp.joints[i].rotate, cpp.joints[i].scale);
    }
    // Restore the joints list to the original rest pose for the retargeting bake
    cpp.joints = saved_rest_pose;

    // 2. Warp Mesh
    // pos_new = sum(w * NewWorldBind * OldIBM * pos_old)
    // norm_new = sum(w * (OldWorldBind * NewIBM)^T * norm_old)
    for (size_t i = 0; i < cpp.positions.size() / 3; ++i) {
        float pos[3] = { cpp.positions[i*3], cpp.positions[i*3+1], cpp.positions[i*3+2] };
        float norm[3] = { cpp.normals[i*3], cpp.normals[i*3+1], cpp.normals[i*3+2] };
        float res_pos[3] = {0,0,0}, res_norm[3] = {0,0,0};
        
        for (int k = 0; k < 4; ++k) {
            float w = cpp.weights_0[i*4+k];
            if (w <= 0) continue;
            int ji = cpp.joints_0[i*4+k];
            
            mat4 warp = mat4_mul(new_world_binds[ji], old_ibms[ji]);
            mat4 nwarp = mat4_transpose(mat4_mul(old_world_binds[ji], new_ibms[ji]));
            
            float p[3], n[3];
            mat4_mul_vec3(warp, pos, p);
            mat4_mul_vec3(nwarp, norm, n);
            for(int c=0; c<3; ++c) {
                res_pos[c] += p[c] * w;
                res_norm[c] += n[c] * w;
            }
        }
        memcpy(&cpp.positions[i*3], res_pos, 12);
        float len = sqrtf(res_norm[0]*res_norm[0] + res_norm[1]*res_norm[1] + res_norm[2]*res_norm[2]);
        if (len > 1e-6f) {
            for(int c=0; c<3; ++c) cpp.normals[i*3+c] = res_norm[c] / len;
        }
    }

    // 3. Retarget Animations
    for (size_t a = 0; a < cpp.animations.size(); ++a) {
        // Skip retargeting for the new bind itself, or the special "original_bind" animation
        if (a == (size_t)pose_anim_idx || cpp.animations[a].name == "original_bind") continue;

        // Collect ALL unique timestamps across ALL bones to create a uniform timeline for this animation
        std::set<double> timestamps;
        for(const auto& b : cpp.animations[a].bones) {
            for(double t : b.translation.times) timestamps.insert(t);
            for(double t : b.rotation.times) timestamps.insert(t);
            for(double t : b.scale.times) timestamps.insert(t);
        }
        if (timestamps.empty()) timestamps.insert(0.0);

        std::vector<double> timeline(timestamps.begin(), timestamps.end());

        // Cache the evaluation of the OLD animation before we clear the tracks
        std::map<double, std::vector<Pose>> old_local_bake;
        for (double t : timeline) {
            std::vector<Pose> old_local_poses;
            cpp.evaluate_animation(a, t, old_local_poses);
            old_local_bake[t] = old_local_poses;
        }

        // Now we can safely overwrite the tracks
        for (size_t i = 0; i < cpp.joints.size(); ++i) {
            BoneAnim& ba = cpp.animations[a].bones[i];

            ba.translation.times = timeline;
            ba.rotation.times = timeline;
            ba.scale.times = timeline;
            ba.translation.values.assign(timeline.size() * 3, 0.0f);
            ba.rotation.values.assign(timeline.size() * 4, 0.0f);
            ba.scale.values.assign(timeline.size() * 3, 0.0f);

            for (size_t k = 0; k < timeline.size(); ++k) {
                double t = timeline[k];
                
                // Get old local pose from cache
                const Pose& old_pose = old_local_bake[t][i];
                
                // We use the original local animation values. 
                // Since the mesh was warped and IBMs updated, using the original 
                // local TRS guarantees perfect world-space deformation invariance.
                memcpy(&ba.translation.values[k * 3], old_pose.translate, 12);
                memcpy(&ba.scale.values[k * 3], old_pose.scale, 12);
                
                float ro[4] = {old_pose.rotate[0], old_pose.rotate[1], old_pose.rotate[2], old_pose.rotate[3]};
                
                // Shortest path correction relative to previous key to prevent twisting
                if (k > 0) {
                    float* prev_q = &ba.rotation.values[(k - 1) * 4];
                    float dot = ro[0]*prev_q[0] + ro[1]*prev_q[1] + ro[2]*prev_q[2] + ro[3]*prev_q[3];
                    if (dot < 0) { ro[0]=-ro[0]; ro[1]=-ro[1]; ro[2]=-ro[2]; ro[3]=-ro[3]; }
                }
                memcpy(&ba.rotation.values[k * 4], ro, 16);
            }
        }
    }

    // 4. Finally update Model Rest Pose and IBMs
    for (size_t i = 0; i < cpp.joints.size(); ++i) {
        memcpy(cpp.joints[i].translate, target_poses[i].translate, 12);
        memcpy(cpp.joints[i].rotate, target_poses[i].rotate, 16);
        memcpy(cpp.joints[i].scale, target_poses[i].scale, 12);
    }
    cpp.ibms = new_ibms;

    // Sync back to C
    gs_model* new_c = model_cpp_to_c(cpp);
    gs_model temp = *model;
    *model = *new_c;
    *new_c = temp;
    gsk_free_model(new_c);

    return true;
}

extern "C" bool gsk_convert_orientation(gs_model* model, gs_coord_system target_orientation, gs_winding_order target_winding) {
    if (!model) return false;
    
    // 1. Create a local C++ Model copy of the C model.
    Model cpp = model_c_to_cpp(model);
    
    // 2. Call convert_orientation(cpp, target_orientation, target_winding) on that copy.
    if (!convert_orientation(cpp, target_orientation, target_winding)) return false;

    // 3. Update model metadata
    model->orientation = cpp.orientation;
    model->winding = cpp.winding;
    model->has_bounds = cpp.has_bounds;
    memcpy(model->mins, cpp.mins, sizeof(model->mins));
    memcpy(model->maxs, cpp.maxs, sizeof(model->maxs));
    model->radius = cpp.radius;
    model->xyradius = cpp.xyradius;

    // 4. Manually copy back the updated values from the C++ Model's vectors into the original C model's pointers.
    
    // Joints (TRS)
    if (model->num_joints > 0 && model->joints) {
        for (uint32_t i = 0; i < model->num_joints; ++i) {
            memcpy(model->joints[i].translate, cpp.joints[i].translate, 3 * sizeof(float));
            memcpy(model->joints[i].rotate, cpp.joints[i].rotate, 4 * sizeof(float));
            memcpy(model->joints[i].scale, cpp.joints[i].scale, 3 * sizeof(float));
        }
    }

    // Vertex data
    if (model->num_vertices > 0) {
        if (model->positions && !cpp.positions.empty()) {
            memcpy(model->positions, cpp.positions.data(), cpp.positions.size() * sizeof(float));
        }
        if (model->normals && !cpp.normals.empty()) {
            memcpy(model->normals, cpp.normals.data(), cpp.normals.size() * sizeof(float));
        }
        if (model->tangents && !cpp.tangents.empty()) {
            memcpy(model->tangents, cpp.tangents.data(), cpp.tangents.size() * sizeof(float));
        }
    }

    // Morph Targets
    for (uint32_t i = 0; i < model->num_morph_targets; ++i) {
        if (model->morph_targets[i].positions && !cpp.morph_targets[i].positions.empty()) {
            memcpy(model->morph_targets[i].positions, cpp.morph_targets[i].positions.data(), cpp.morph_targets[i].positions.size() * sizeof(float));
        }
        if (model->morph_targets[i].normals && !cpp.morph_targets[i].normals.empty()) {
            memcpy(model->morph_targets[i].normals, cpp.morph_targets[i].normals.data(), cpp.morph_targets[i].normals.size() * sizeof(float));
        }
    }

    // Indices
    if (model->num_indices > 0 && model->indices && !cpp.indices.empty()) {
        memcpy(model->indices, cpp.indices.data(), cpp.indices.size() * sizeof(uint32_t));
    }

    // Animation Tracks
    for (uint32_t i = 0; i < model->num_animations; ++i) {
        for (uint32_t j = 0; j < model->animations[i].num_bones; ++j) {
            gs_bone_anim& dst_bone = model->animations[i].bones[j];
            const BoneAnim& src_bone = cpp.animations[i].bones[j];
            
            if (dst_bone.translation.values && !src_bone.translation.values.empty()) {
                memcpy(dst_bone.translation.values, src_bone.translation.values.data(), src_bone.translation.values.size() * sizeof(float));
            }
            if (dst_bone.rotation.values && !src_bone.rotation.values.empty()) {
                memcpy(dst_bone.rotation.values, src_bone.rotation.values.data(), src_bone.rotation.values.size() * sizeof(float));
            }
            if (dst_bone.scale.values && !src_bone.scale.values.empty()) {
                memcpy(dst_bone.scale.values, src_bone.scale.values.data(), src_bone.scale.values.size() * sizeof(float));
            }
        }
    }

    // Matrices
    if (model->world_matrices && !cpp.world_matrices.empty()) {
        memcpy(model->world_matrices, cpp.world_matrices.data(), cpp.world_matrices.size() * sizeof(gs_mat4));
    }
    if (model->ibms && !cpp.ibms.empty()) {
        memcpy(model->ibms, cpp.ibms.data(), cpp.ibms.size() * sizeof(gs_mat4));
    }
    if (model->computed_ibms && !cpp.computed_ibms.empty()) {
        memcpy(model->computed_ibms, cpp.computed_ibms.data(), cpp.computed_ibms.size() * sizeof(gs_mat4));
    }

    return true;
}
