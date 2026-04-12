#include "fbx_loader.h"
#include "ufbx.h"
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>
#include <fstream>

bool load_fbx(const char* path, Model& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    size_t size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> buf(size);
    f.read(buf.data(), size);
    f.close();

    return load_fbx_from_memory(buf.data(), size, out);
}

bool load_fbx_from_memory(const void* data, size_t size, Model& out) {
    ufbx_load_opts opts = { 0 };
    opts.target_unit_meters = 1.0f;
    // We do NOT set target_axes here to let ufbx load the NATIVE coordinate system
    opts.allow_unsafe = true;

    ufbx_error error;
    ufbx_scene* scene = ufbx_load_memory(data, size, &opts, &error);

    if (!scene) {
        std::cerr << "Failed to load FBX from memory: " << error.description.data << std::endl;
        return false;
    }

    // 0. Extract Textures
    std::map<ufbx_texture*, std::string> texture_to_path;
    for (size_t i = 0; i < scene->textures.count; ++i) {
        ufbx_texture* tex = scene->textures[i];
        if (tex->type != UFBX_TEXTURE_FILE) continue;

        ufbx_blob content = { 0 };
        if (tex->content.size > 0) {
            content = tex->content;
        } else if (tex->video && tex->video->content.size > 0) {
            content = tex->video->content;
        }

        std::string virtual_path;
        if (content.size > 0) {
            if (tex->name.length > 0) {
                virtual_path = "embedded://" + std::string(tex->name.data);
            } else if (tex->filename.length > 0) {
                std::string full_path = tex->filename.data;
                size_t last_slash = full_path.find_last_of("\\/");
                if (last_slash != std::string::npos) {
                    virtual_path = "embedded://" + full_path.substr(last_slash + 1);
                } else {
                    virtual_path = "embedded://" + full_path;
                }
            } else {
                virtual_path = "embedded://texture_" + std::to_string(i);
            }

            // Ensure uniqueness
            std::string base_path = virtual_path;
            int counter = 1;
            bool unique = false;
            while (!unique) {
                unique = true;
                for (auto const& it : texture_to_path) {
                    if (it.second == virtual_path) {
                        unique = false;
                        virtual_path = base_path + "_" + std::to_string(counter++);
                        break;
                    }
                }
            }

            TextureBuffer tb;
            tb.original_path = virtual_path;
            tb.data.resize(content.size);
            memcpy(tb.data.data(), content.data, content.size);
            out.textures.push_back(tb);
            texture_to_path[tex] = virtual_path;
        } else {
            // Not embedded, use the filename as is or basename if possible
            texture_to_path[tex] = tex->filename.data;
        }
    }

    // Determine orientation from scene settings
    ufbx_coordinate_axes axes = scene->settings.axes;
    if (axes.up == UFBX_COORDINATE_AXIS_POSITIVE_Y) {
        if (axes.front == UFBX_COORDINATE_AXIS_POSITIVE_Z) out.orientation = GS_Y_UP_RIGHTHANDED;
        else out.orientation = GS_Y_UP_LEFTHANDED;
    } else if (axes.up == UFBX_COORDINATE_AXIS_POSITIVE_Z) {
        if (axes.front == UFBX_COORDINATE_AXIS_NEGATIVE_Y) out.orientation = GS_Z_UP_RIGHTHANDED;
        else if (axes.front == UFBX_COORDINATE_AXIS_POSITIVE_X) out.orientation = GS_Z_UP_RIGHTHANDED_X_FWD;
        else out.orientation = GS_Z_UP_LEFTHANDED;
    } else if (axes.up == UFBX_COORDINATE_AXIS_POSITIVE_X) {
        if (axes.front == UFBX_COORDINATE_AXIS_POSITIVE_Z) out.orientation = GS_X_UP_RIGHTHANDED;
        else out.orientation = GS_X_UP_LEFTHANDED;
    } else {
        out.orientation = GS_Y_UP_RIGHTHANDED; // Fallback
    }
    out.winding = GS_WINDING_CCW; // FBX is CCW by convention

    // Phase 1 - Extract Skeleton (Full Hierarchy)
    // We add an explicit IDENTITY ROOT at index 0.
    // This ensures that unskinned meshes baked to world space can be attached to an 
    // identity transform, preventing double-transformation from FBX root scaling.
    std::map<ufbx_node*, int> node_to_joint;
    
    Joint root;
    root.name = "world_root";
    root.parent = -1;
    root.translate[0] = root.translate[1] = root.translate[2] = 0.0f;
    root.rotate[0] = root.rotate[1] = root.rotate[2] = 0.0f; root.rotate[3] = 1.0f;
    root.scale[0] = root.scale[1] = root.scale[2] = 1.0f;
    out.joints.push_back(root);

    for (size_t i = 0; i < scene->nodes.count; ++i) {
        ufbx_node* node = scene->nodes[i];
        Joint j;
        j.name = node->name.data;
        j.parent = 0; // Default parent is our new world root
        
        float t[3] = { (float)node->local_transform.translation.x, (float)node->local_transform.translation.y, (float)node->local_transform.translation.z };
        float r[4] = { (float)node->local_transform.rotation.x, (float)node->local_transform.rotation.y, (float)node->local_transform.rotation.z, (float)node->local_transform.rotation.w };
        float s[3] = { (float)node->local_transform.scale.x, (float)node->local_transform.scale.y, (float)node->local_transform.scale.z };
        
        stabilize_trs(t, r, s);

        memcpy(j.translate, t, 12);
        memcpy(j.rotate, r, 16);
        memcpy(j.scale, s, 12);
        
        node_to_joint[node] = (int)out.joints.size();
        out.joints.push_back(j);
    }

    // Set parents
    for (size_t i = 0; i < scene->nodes.count; ++i) {
        ufbx_node* node = scene->nodes[i];
        if (node_to_joint.count(node)) {
            int ji = node_to_joint[node];
            if (node->parent && node_to_joint.count(node->parent)) {
                out.joints[ji].parent = node_to_joint[node->parent];
            } else {
                out.joints[ji].parent = 0; // Attach scene-level nodes to world_root
            }
        }
    }

    // Pre-calculate bind pose world matrices for mesh baking
    out.compute_bind_pose();

    // Phase 2 - Extract Meshes, Materials, and Skinning
    std::map<std::string, int> material_map;
    for (size_t i = 0; i < scene->meshes.count; ++i) {
        ufbx_mesh* fmesh = scene->meshes[i];
        if (fmesh->num_indices == 0) continue;

        // Find the first instance of this mesh to determine its world-space attachment
        ufbx_node* mesh_node = nullptr;
        if (fmesh->instances.count > 0) mesh_node = fmesh->instances.data[0];

        // Unified Model-Space Skinning:
        // All vertices are stored in World Space at Rest Pose.
        ufbx_matrix geo_to_world;
        if (mesh_node) {
            geo_to_world = mesh_node->geometry_to_world;
        } else {
            memset(&geo_to_world, 0, sizeof(geo_to_world));
            geo_to_world.m00 = geo_to_world.m11 = geo_to_world.m22 = 1.0f;
        }

        // For static meshes, we assign them to world_root (0) with weight 1.0.
        // Since vertices are baked to world space, and world_root is identity, 
        // the shader will render them exactly at their baked world positions.
        int default_joint = 0; 

        Mesh out_mesh;
        out_mesh.name = fmesh->name.data;
        out_mesh.first_vertex = (uint32_t)out.positions.size() / 3;
        out_mesh.first_triangle = (uint32_t)out.indices.size() / 3;
        uint32_t vertex_counter = 0;

        std::string mat_name = "default";
        if (fmesh->materials.count > 0 && fmesh->materials[0]) {
            mat_name = fmesh->materials[0]->name.data;
        }

        if (material_map.find(mat_name) == material_map.end()) {
            material_map[mat_name] = (int)out.materials.size();
            Material mat;
            mat.name = mat_name;
            ufbx_material* fmat = (fmesh->materials.count > 0) ? fmesh->materials[0] : nullptr;
            if (fmat) {
                auto get_tex_path = [&](ufbx_texture* tex) -> std::string {
                    if (!tex) return "";
                    if (texture_to_path.count(tex)) return texture_to_path[tex];
                    return tex->filename.data;
                };

                mat.material_type = fmat->features.pbr.enabled ? 0 : 1;

                mat.color_map = get_tex_path(fmat->pbr.base_color.texture);
                if (mat.color_map.empty()) mat.color_map = get_tex_path(fmat->fbx.diffuse_color.texture);

                mat.normal_map = get_tex_path(fmat->pbr.normal_map.texture);
                if (mat.normal_map.empty()) mat.normal_map = get_tex_path(fmat->fbx.normal_map.texture);
                if (mat.normal_map.empty()) mat.normal_map = get_tex_path(fmat->fbx.bump.texture);

                mat.metallic_map = get_tex_path(fmat->pbr.metalness.texture);
                if (mat.metallic_map.empty()) mat.metallic_map = get_tex_path(fmat->fbx.reflection_factor.texture);

                mat.roughness_map = get_tex_path(fmat->pbr.roughness.texture);
                if (mat.roughness_map.empty()) mat.roughness_map = get_tex_path(fmat->pbr.glossiness.texture);

                mat.emissive_map = get_tex_path(fmat->pbr.emission_color.texture);
                if (mat.emissive_map.empty()) mat.emissive_map = get_tex_path(fmat->fbx.emission_color.texture);

                // Proactive keyword probing: handles FBX files with non-standard slot connections
                auto find_keyword = [&](const char* kw) -> std::string {
                    for (size_t ti = 0; ti < fmat->textures.count; ++ti) {
                        std::string p = get_tex_path(fmat->textures[ti].texture);
                        std::string lp = p;
                        for (char& c : lp) c = (char)std::tolower((unsigned char)c);
                        if (lp.find(kw) != std::string::npos) return p;
                    }
                    return "";
                };

                if (mat.metallic_map.empty()) mat.metallic_map = find_keyword("metal");
                if (mat.roughness_map.empty()) {
                    mat.roughness_map = find_keyword("rough");
                    if (mat.roughness_map.empty()) mat.roughness_map = find_keyword("gloss");
                }
                if (mat.normal_map.empty()) {
                    mat.normal_map = find_keyword("normal");
                    if (mat.normal_map.empty()) mat.normal_map = find_keyword("bump");
                }
                if (mat.color_map.empty()) {
                    mat.color_map = find_keyword("albedo");
                    if (mat.color_map.empty()) mat.color_map = find_keyword("base");
                    if (mat.color_map.empty()) mat.color_map = find_keyword("diff");
                }
                if (mat.emissive_map.empty()) mat.emissive_map = find_keyword("emis");

                mat.occlusion_map = get_tex_path(fmat->pbr.ambient_occlusion.texture);
                mat.opacity_map = get_tex_path(fmat->pbr.opacity.texture);
                if (mat.opacity_map.empty()) mat.opacity_map = get_tex_path(fmat->fbx.transparency_color.texture);

                // Fallback: if not natively PBR, check suffixes or map population
                if (mat.material_type != 0) {
                    if (is_pbr_suffix(mat.color_map) || is_pbr_suffix(mat.normal_map) || 
                        is_pbr_suffix(mat.metallic_map) || is_pbr_suffix(mat.roughness_map) ||
                        !mat.metallic_map.empty() || !mat.roughness_map.empty()) {
                        mat.material_type = 0;
                    }
                }

                mat.base_color[0] = (float)fmat->pbr.base_color.value_vec4.x;
                mat.base_color[1] = (float)fmat->pbr.base_color.value_vec4.y;
                mat.base_color[2] = (float)fmat->pbr.base_color.value_vec4.z;
                mat.base_color[3] = (float)fmat->pbr.base_color.value_vec4.w;
                mat.metallic_factor = (float)fmat->pbr.metalness.value_real;
                mat.roughness_factor = (float)fmat->pbr.roughness.value_real;
                mat.emissive_color[0] = (float)fmat->fbx.emission_color.value_vec3.x;
                mat.emissive_color[1] = (float)fmat->fbx.emission_color.value_vec3.y;
                mat.emissive_color[2] = (float)fmat->fbx.emission_color.value_vec3.z;
            }
            out.materials.push_back(mat);
        }
        out_mesh.material_idx = material_map[mat_name];

        std::vector<uint32_t> tri_indices(std::max((size_t)1, fmesh->max_face_triangles) * 3);

        for (size_t face_i = 0; face_i < fmesh->num_faces; ++face_i) {
            ufbx_face face = fmesh->faces.data[face_i];
            uint32_t num_tris = ufbx_triangulate_face(tri_indices.data(), tri_indices.size(), fmesh, face);

            for (uint32_t k = 0; k < num_tris * 3; ++k) {
                uint32_t fi = tri_indices[k];
                uint32_t idx = fmesh->vertex_indices[fi];
                
                // Bake vertex to WORLD space (Rest Pose)
                ufbx_vec3 pos = ufbx_transform_position(&geo_to_world, fmesh->vertices[idx]);
                out.positions.push_back((float)pos.x);
                out.positions.push_back((float)pos.y);
                out.positions.push_back((float)pos.z);
                
                if (fmesh->vertex_normal.exists) {
                    ufbx_vec3 n_raw = ufbx_get_vertex_vec3(&fmesh->vertex_normal, fi);
                    ufbx_vec3 n = ufbx_transform_direction(&geo_to_world, n_raw);
                    out.normals.push_back((float)n.x);
                    out.normals.push_back((float)n.y);
                    out.normals.push_back((float)n.z);
                } else {
                    for (int n = 0; n < 3; ++n) out.normals.push_back(0.0f);
                }
                
                if (fmesh->vertex_uv.exists) {
                    ufbx_vec2 uv = ufbx_get_vertex_vec2(&fmesh->vertex_uv, fi);
                    out.texcoords.push_back((float)uv.x);
                    out.texcoords.push_back(1.0f - (float)uv.y);
                } else {
                    for (int u = 0; u < 2; ++u) out.texcoords.push_back(0.0f);
                }

                float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
                uint8_t joints[4] = { 0, 0, 0, 0 };

                if (fmesh->skin_deformers.count > 0) {
                    ufbx_skin_deformer* skin = fmesh->skin_deformers[0];
                    if (skin && skin->max_weights_per_vertex > 0) {
                        ufbx_skin_vertex s_vert = skin->vertices[idx];
                        float total_w = 0.0f;
                        int actual_weights = std::min((int)s_vert.num_weights, 4);
                        for (int w = 0; w < actual_weights; ++w) {
                            ufbx_skin_weight sw = skin->weights[s_vert.weight_begin + w];
                            ufbx_node* joint_node = skin->clusters[sw.cluster_index]->bone_node;
                            if (joint_node && node_to_joint.count(joint_node)) {
                                joints[w] = (uint8_t)node_to_joint[joint_node];
                                weights[w] = (float)sw.weight;
                                total_w += weights[w];
                            }
                        }
                        if (total_w > 1e-6f) {
                            for (int w = 0; w < 4; ++w) weights[w] /= total_w;
                        } else {
                            joints[0] = (uint8_t)default_joint;
                            weights[0] = 1.0f;
                        }
                    }
                } else {
                    joints[0] = (uint8_t)default_joint;
                    weights[0] = 1.0f;
                }

                for (int w = 0; w < 4; ++w) {
                    if (joints[w] >= 255) joints[w] = 0;
                    out.joints_0.push_back(joints[w]);
                    out.weights_0.push_back(weights[w]);
                }
                out.indices.push_back((uint32_t)(out_mesh.first_vertex + vertex_counter));
                vertex_counter++;
            }
        }

        out_mesh.num_vertexes = vertex_counter;
        out_mesh.num_triangles = vertex_counter / 3;
        out.meshes.push_back(out_mesh);
    }

    // IBMs: Use world-space computed IBMs.
    out.ibms = out.computed_ibms;

    // Phase 3 - Animations (Extract Sparse Baked Data)
    for (size_t si = 0; si < scene->anim_stacks.count; ++si) {
        ufbx_anim_stack* stack = scene->anim_stacks[si];
        ufbx_bake_opts bake_opts = { 0 };
        ufbx_baked_anim* baked_anim = ufbx_bake_anim(scene, stack->anim, &bake_opts, &error);
        if (!baked_anim) continue;

        AnimationDef ad;
        ad.name = stack->name.data;
        ad.duration = baked_anim->playback_duration;
        ad.bones.resize(out.joints.size());

        for (size_t ji = 0; ji < out.joints.size(); ++ji) {
            ufbx_node* node = nullptr;
            for (auto const& [n, idx] : node_to_joint) {
                if (idx == (int)ji) { node = n; break; }
            }
            if (!node) continue;

            ufbx_baked_node* baked_node = ufbx_find_baked_node(baked_anim, node);
            if (!baked_node) continue;

            BoneAnim& ba = ad.bones[ji];
            
            // Extract Translations
            for (size_t k = 0; k < baked_node->translation_keys.count; ++k) {
                ufbx_baked_vec3 key = baked_node->translation_keys.data[k];
                ba.translation.times.push_back(key.time);
                ba.translation.values.push_back((float)key.value.x);
                ba.translation.values.push_back((float)key.value.y);
                ba.translation.values.push_back((float)key.value.z);
            }

            // Extract Rotations
            float prev_q[4] = {0,0,0,1};
            for (size_t k = 0; k < baked_node->rotation_keys.count; ++k) {
                ufbx_baked_quat key = baked_node->rotation_keys.data[k];
                float q[4] = {(float)key.value.x, (float)key.value.y, (float)key.value.z, (float)key.value.w};
                if (k > 0) {
                    float dot = q[0]*prev_q[0] + q[1]*prev_q[1] + q[2]*prev_q[2] + q[3]*prev_q[3];
                    if (dot < 0) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
                }
                memcpy(prev_q, q, 16);
                ba.rotation.times.push_back(key.time);
                ba.rotation.values.push_back(q[0]);
                ba.rotation.values.push_back(q[1]);
                ba.rotation.values.push_back(q[2]);
                ba.rotation.values.push_back(q[3]);
            }

            // Extract Scale
            for (size_t k = 0; k < baked_node->scale_keys.count; ++k) {
                ufbx_baked_vec3 key = baked_node->scale_keys.data[k];
                ba.scale.times.push_back(key.time);
                ba.scale.values.push_back((float)key.value.x);
                ba.scale.values.push_back((float)key.value.y);
                ba.scale.values.push_back((float)key.value.z);
            }
        }
        out.animations.push_back(ad);
        ufbx_free_baked_anim(baked_anim);
    }

    ufbx_free_scene(scene);
    return true;
}
