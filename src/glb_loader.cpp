#include "glb_loader.h"
#include "cgltf.h"
#include <iostream>
#include <vector>
#include <map>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fstream>

bool load_glb(const char* path, Model& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    size_t size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> buf(size);
    f.read(buf.data(), size);
    f.close();

    return load_glb_from_memory(buf.data(), size, out);
}

bool load_glb_from_memory(const void* data, size_t size, Model& out) {
    cgltf_options options = {};
    cgltf_data* gdata = nullptr;
    cgltf_result result = cgltf_parse(&options, data, size, &gdata);
    if (result != cgltf_result_success) return false;

    // For memory loading, we assume it's a self-contained GLB or the data already contains buffers.
    if (gdata->file_type == cgltf_file_type_glb) {
        result = cgltf_load_buffers(&options, gdata, nullptr);
    }

    std::map<cgltf_node*, int> node_to_joint;
    
    // 1. Load Joints
    // We add an explicit IDENTITY ROOT at index 0.
    Joint world_root;
    world_root.name = "world_root";
    world_root.parent = -1;
    world_root.translate[0] = world_root.translate[1] = world_root.translate[2] = 0.0f;
    world_root.rotate[0] = world_root.rotate[1] = world_root.rotate[2] = 0.0f; world_root.rotate[3] = 1.0f;
    world_root.scale[0] = world_root.scale[1] = world_root.scale[2] = 1.0f;
    out.joints.push_back(world_root);

    for (size_t i = 0; i < gdata->nodes_count; ++i) {
        cgltf_node* node = &gdata->nodes[i];
        if (node->camera || node->light) continue; 
        
        int ji = (int)out.joints.size();
        node_to_joint[node] = ji;
        Joint j;
        j.name = node->name ? node->name : "node_" + std::to_string(i);
        j.parent = 0; // Default parent is world_root
        
        if (node->has_matrix) {
            mat4 m;
            memcpy(m.m, node->matrix, 64);
            mat4_decompose(m, j.translate, j.rotate, j.scale);
        } else {
            if (node->has_translation) memcpy(j.translate, node->translation, 12);
            else { j.translate[0] = j.translate[1] = j.translate[2] = 0.0f; }
            
            if (node->has_rotation) memcpy(j.rotate, node->rotation, 16);
            else { j.rotate[0] = j.rotate[1] = j.rotate[2] = 0.0f; j.rotate[3] = 1.0f; }
            
            if (node->has_scale) memcpy(j.scale, node->scale, 12);
            else { j.scale[0] = j.scale[1] = j.scale[2] = 1.0f; }
        }
        
        // Diagnostic: log any non-identity scale
        if (std::abs(j.scale[0]-1.0f) > 0.01f || std::abs(j.scale[1]-1.0f) > 0.01f || std::abs(j.scale[2]-1.0f) > 0.01f) {
            printf("Joint '%s' has scale: %.3f, %.3f, %.3f\n", j.name.c_str(), j.scale[0], j.scale[1], j.scale[2]);
        }
        
        out.joints.push_back(j);
    }
    
    // Set parents
    for (size_t i = 0; i < gdata->nodes_count; ++i) {
        cgltf_node* node = &gdata->nodes[i];
        if (node_to_joint.count(node)) {
            int ji = node_to_joint[node];
            if (node->parent && node_to_joint.count(node->parent)) {
                out.joints[ji].parent = node_to_joint[node->parent];
            } else {
                out.joints[ji].parent = 0; // Root nodes attach to world_root
            }
        }
    }

    // 2. Load Meshes
    for (size_t i = 0; i < gdata->meshes_count; ++i) {
        cgltf_mesh* mesh = &gdata->meshes[i];
        for (size_t j = 0; j < mesh->primitives_count; ++j) {
            cgltf_primitive* prim = &mesh->primitives[j];
            if (prim->type != cgltf_primitive_type_triangles) continue;

            Mesh m;
            m.name = mesh->name ? mesh->name : "mesh";

            std::string mat_name = "default";
            cgltf_material* cmat = prim->material;
            if (cmat && cmat->name) mat_name = cmat->name;
            else if (cmat) mat_name = "material_" + std::to_string(i);

            int mat_idx = -1;
            for (int m_idx = 0; m_idx < (int)out.materials.size(); ++m_idx) {
                if (out.materials[m_idx].name == mat_name) {
                    mat_idx = m_idx;
                    break;
                }
            }

            if (mat_idx == -1) {
                mat_idx = (int)out.materials.size();
                Material mat;
                mat.name = mat_name;
                if (cmat) {
                    if (cmat->has_pbr_metallic_roughness) {
                        mat.material_type = 0;
                        memcpy(mat.base_color, cmat->pbr_metallic_roughness.base_color_factor, 16);
                        mat.metallic_factor = cmat->pbr_metallic_roughness.metallic_factor;
                        mat.roughness_factor = cmat->pbr_metallic_roughness.roughness_factor;
                        if (cmat->pbr_metallic_roughness.base_color_texture.texture && cmat->pbr_metallic_roughness.base_color_texture.texture->image) {
                            if (cmat->pbr_metallic_roughness.base_color_texture.texture->image->uri)
                                mat.color_map = cmat->pbr_metallic_roughness.base_color_texture.texture->image->uri;
                        }
                    }
                    if (cmat->normal_texture.texture && cmat->normal_texture.texture->image) {
                        if (cmat->normal_texture.texture->image->uri)
                            mat.normal_map = cmat->normal_texture.texture->image->uri;
                    }
                    memcpy(mat.emissive_color, cmat->emissive_factor, 12);
                }
                out.materials.push_back(mat);
            }
            m.material_idx = mat_idx;

            m.first_vertex = (uint32_t)out.positions.size() / 3;
            m.first_triangle = (uint32_t)out.indices.size() / 3;
            uint32_t num_verts = 0;

            // Find the skin associated with this mesh/node
            cgltf_skin* skin = nullptr;
            for (size_t n = 0; n < gdata->nodes_count; ++n) {
                if (gdata->nodes[n].mesh == mesh && gdata->nodes[n].skin) {
                    skin = gdata->nodes[n].skin;
                    break;
                }
            }
            if (!skin && gdata->skins_count > 0) skin = &gdata->skins[0];

            // 2.1 Find vertex count first to ensure all buffers are grown equally
            for (size_t k = 0; k < prim->attributes_count; ++k) {
                if (prim->attributes[k].type == cgltf_attribute_type_position) {
                    num_verts = (uint32_t)prim->attributes[k].data->count;
                    break;
                }
            }
            if (num_verts == 0) continue;

            size_t v_start = out.positions.size() / 3;
            out.positions.resize((v_start + num_verts) * 3, 0.0f);
            out.normals.resize((v_start + num_verts) * 3, 0.0f);
            out.texcoords.resize((v_start + num_verts) * 2, 0.0f);
            out.joints_0.resize((v_start + num_verts) * 4, 0);
            out.weights_0.resize((v_start + num_verts) * 4, 0.0f);

            // 2.2 Pre-initialize weights/joints to handle unskinned primitives
            int default_joint = 0;
            mat4 mesh_world_matrix = mat4_identity();

            for (size_t n = 0; n < gdata->nodes_count; ++n) {
                if (gdata->nodes[n].mesh == mesh) {
                    if (node_to_joint.count(&gdata->nodes[n])) {
                        default_joint = node_to_joint[&gdata->nodes[n]];
                        // Extract node world transform for baking
                        float t[3], r[4], s[3];
                        cgltf_node_transform_world(&gdata->nodes[n], (float*)&mesh_world_matrix);
                    }
                    break;
                }
            }

            for (size_t v = 0; v < num_verts; ++v) {
                out.joints_0[(v_start + v)*4 + 0] = (uint8_t)default_joint;
                out.weights_0[(v_start + v)*4 + 0] = 1.0f;
            }

            // Attributes
            for (size_t k = 0; k < prim->attributes_count; ++k) {
                cgltf_attribute* attr = &prim->attributes[k];
                if (attr->type == cgltf_attribute_type_position) {
                    for (size_t v = 0; v < num_verts; ++v) {
                        float pos[3];
                        cgltf_accessor_read_float(attr->data, v, pos, 3);
                        
                        // Transform to WORLD space if not already (Model-Space Skinning)
                        float world_pos[3];
                        mat4_mul_vec3(mesh_world_matrix, pos, world_pos);
                        memcpy(&out.positions[(v_start + v) * 3], world_pos, 12);
                    }
                    m.num_vertexes = num_verts;
                } else if (attr->type == cgltf_attribute_type_normal) {
                    for (size_t v = 0; v < num_verts; ++v) {
                        float norm[3];
                        cgltf_accessor_read_float(attr->data, v, norm, 3);
                        
                        // Transform direction to WORLD space
                        // Using simplified normal transform (assumes uniform scale for now)
                        float world_norm[3];
                        mat4 norm_mat = mesh_world_matrix;
                        norm_mat.m[12] = norm_mat.m[13] = norm_mat.m[14] = 0.0f; // Remove translation
                        mat4_mul_vec3(norm_mat, norm, world_norm);
                        
                        float len = sqrtf(world_norm[0]*world_norm[0] + world_norm[1]*world_norm[1] + world_norm[2]*world_norm[2]);
                        if (len > 1e-6f) {
                            world_norm[0] /= len; world_norm[1] /= len; world_norm[2] /= len;
                        }
                        memcpy(&out.normals[(v_start + v) * 3], world_norm, 12);
                    }
                } else if (attr->type == cgltf_attribute_type_texcoord) {
                    for (size_t v = 0; v < num_verts; ++v)
                        cgltf_accessor_read_float(attr->data, v, &out.texcoords[(v_start + v) * 2], 2);
                } else if (attr->type == cgltf_attribute_type_joints) {
                    for (size_t v = 0; v < num_verts; ++v) {
                        uint32_t joints[4];
                        cgltf_accessor_read_uint(attr->data, v, joints, 4);
                        for(int c=0; c<4; ++c) {
                            uint32_t final_joint = joints[c];
                            if (skin && joints[c] < skin->joints_count) {
                                cgltf_node* jnode = skin->joints[joints[c]];
                                final_joint = (uint32_t)node_to_joint[jnode];
                            }
                            
                            if (final_joint >= 255) final_joint = 0;
                            out.joints_0[(v_start + v)*4+c] = (uint8_t)final_joint;
                        }
                    }
                } else if (attr->type == cgltf_attribute_type_weights) {
                    for (size_t v = 0; v < num_verts; ++v) {
                        float w[4];
                        cgltf_accessor_read_float(attr->data, v, w, 4);
                        float total_w = w[0]+w[1]+w[2]+w[3];
                        if (total_w > 1e-6f) {
                            for(int c=0; c<4; ++c) out.weights_0[(v_start + v)*4 + c] = w[c]/total_w;
                        } else {
                            out.weights_0[(v_start + v)*4+0] = 1.0f;
                            for(int c=1; c<4; ++c) out.weights_0[(v_start + v)*4 + c] = 0.0f;
                        }
                    }
                }
            }

            // Indices
            if (prim->indices) {
                size_t start = out.indices.size();
                out.indices.resize(start + prim->indices->count);
                for (size_t v = 0; v < prim->indices->count; ++v)
                    out.indices[start + v] = (uint32_t)(v_start + cgltf_accessor_read_index(prim->indices, v));
                m.num_triangles = (uint32_t)prim->indices->count / 3;
            }

            printf("  GLB Mesh %d: \"%s\" %d verts, %d tris, default_joint: %d\n", (int)i, m.name.c_str(), (int)m.num_vertexes, (int)m.num_triangles, default_joint);
            out.meshes.push_back(m);
        }
    }

    // 3. Load Skins (IBMs)
    if (gdata->skins_count > 0) {
        cgltf_skin* skin = &gdata->skins[0]; // Support first skin for now
        if (skin->inverse_bind_matrices) {
            out.ibms.assign(out.joints.size(), mat4_identity());
            for (size_t i = 0; i < skin->joints_count; ++i) {
                cgltf_node* joint_node = skin->joints[i];
                if (node_to_joint.count(joint_node)) {
                    int joint_idx = node_to_joint[joint_node];
                    cgltf_accessor_read_float(skin->inverse_bind_matrices, i, out.ibms[joint_idx].m, 16);
                }
            }
        }
    }

    // 4. Animations
    if (gdata->animations_count > 0 && !out.joints.empty()) {
        for (size_t i = 0; i < gdata->animations_count; ++i) {
            cgltf_animation* anim = &gdata->animations[i];

            AnimationDef ad;
            ad.name = anim->name ? anim->name : "anim_" + std::to_string(i);
            ad.duration = 0.0;
            std::replace(ad.name.begin(), ad.name.end(), ' ', '_');
            ad.bones.resize(out.joints.size());

            for (size_t j = 0; j < anim->channels_count; ++j) {
                cgltf_animation_channel* chan = &anim->channels[j];
                if (!chan->target_node) continue;

                int ji = node_to_joint[chan->target_node];
                BoneAnim& ba = ad.bones[ji];
                AnimChannel* target = nullptr;

                if (chan->target_path == cgltf_animation_path_type_translation) target = &ba.translation;
                else if (chan->target_path == cgltf_animation_path_type_rotation) target = &ba.rotation;
                else if (chan->target_path == cgltf_animation_path_type_scale) target = &ba.scale;

                if (target && chan->sampler->input && chan->sampler->output) {
                    cgltf_accessor* times = chan->sampler->input;
                    cgltf_accessor* values = chan->sampler->output;

                    target->times.resize(times->count);
                    for (size_t k = 0; k < times->count; ++k) {
                        float t;
                        cgltf_accessor_read_float(times, k, &t, 1);
                        target->times[k] = (double)t;
                    }
                    if (!target->times.empty()) {
                        ad.duration = std::max(ad.duration, target->times.back());
                    }
                    int components = (chan->target_path == cgltf_animation_path_type_rotation) ? 4 : 3;
                    target->values.resize(values->count * components);
                    for (size_t k = 0; k < values->count; ++k) {
                        cgltf_accessor_read_float(values, k, &target->values[k * components], components);
                    }
                }
            }
            out.animations.push_back(ad);
        }
    }
    
    out.orientation = GS_Y_UP_RIGHTHANDED;
    out.winding = GS_WINDING_CCW;

    cgltf_free(gdata);
    return true;
}
