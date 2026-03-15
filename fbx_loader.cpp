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
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.allow_unsafe = true;

    ufbx_error error;
    ufbx_scene* scene = ufbx_load_memory(data, size, &opts, &error);

    if (!scene) {
        std::cerr << "Failed to load FBX from memory: " << error.description.data << std::endl;
        return false;
    }

    // Phase 1.5 - Extract Skeleton (Full Hierarchy)
    std::map<ufbx_node*, int> node_to_joint;
    for (size_t i = 0; i < scene->nodes.count; ++i) {
        ufbx_node* node = scene->nodes[i];
        Joint j;
        j.name = node->name.data;
        j.parent = -1;
        
        float t[3] = { (float)node->local_transform.translation.x, (float)node->local_transform.translation.y, (float)node->local_transform.translation.z };
        float r[4] = { (float)node->local_transform.rotation.x, (float)node->local_transform.rotation.y, (float)node->local_transform.rotation.z, (float)node->local_transform.rotation.w };
        
        memcpy(j.translate, t, 12);
        memcpy(j.rotate, r, 16);
        
        j.scale[0] = (float)node->local_transform.scale.x;
        j.scale[1] = (float)node->local_transform.scale.y;
        j.scale[2] = (float)node->local_transform.scale.z;
        
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
            }
        }
    }

    // Phase 2 & 3 - Extract Meshes, Materials, and Skinning
    for (size_t i = 0; i < scene->meshes.count; ++i) {
        ufbx_mesh* fmesh = scene->meshes[i];
        if (fmesh->num_indices == 0) continue;

        Mesh out_mesh;
        out_mesh.name = fmesh->name.data;
        out_mesh.first_vertex = (uint32_t)out.positions.size() / 3;
        out_mesh.first_triangle = (uint32_t)out.indices.size() / 3;
        out_mesh.num_vertexes = (uint32_t)fmesh->num_indices;
        out_mesh.num_triangles = (uint32_t)fmesh->num_indices / 3;
        
        if (fmesh->materials.count > 0 && fmesh->materials[0]) {
            out_mesh.material_name = fmesh->materials[0]->name.data;
        } else {
            out_mesh.material_name = "default";
        }

        for (size_t fi = 0; fi < fmesh->num_indices; ++fi) {
            uint32_t idx = fmesh->vertex_indices[fi];
            out.positions.push_back((float)fmesh->vertices[idx].x);
            out.positions.push_back((float)fmesh->vertices[idx].y);
            out.positions.push_back((float)fmesh->vertices[idx].z);
            
            if (fmesh->vertex_normal.exists) {
                ufbx_vec3 n = ufbx_get_vertex_vec3(&fmesh->vertex_normal, fi);
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
                    }
                }
            }

            for (int k = 0; k < 4; ++k) {
                out.joints_0.push_back(joints[k]);
                out.weights_0.push_back(weights[k]);
            }
            out.indices.push_back((uint32_t)(out_mesh.first_vertex + fi));
        }
        out.meshes.push_back(out_mesh);
    }

    // IBMs
    out.ibms.resize(out.joints.size());
    for (size_t ji = 0; ji < out.joints.size(); ++ji) {
        ufbx_node* node = nullptr;
        for (auto const& [n, idx] : node_to_joint) {
            if (idx == (int)ji) { node = n; break; }
        }
        if (node) {
            ufbx_matrix m = node->node_to_world;
            mat4 mat;
            float* dest = (float*)&mat;
            dest[0] = (float)m.cols[0].x; dest[1] = (float)m.cols[0].y; dest[2] = (float)m.cols[0].z; dest[3] = 0;
            dest[4] = (float)m.cols[1].x; dest[5] = (float)m.cols[1].y; dest[6] = (float)m.cols[1].z; dest[7] = 0;
            dest[8] = (float)m.cols[2].x; dest[9] = (float)m.cols[2].y; dest[10] = (float)m.cols[2].z; dest[11] = 0;
            dest[12] = (float)m.cols[3].x; dest[13] = (float)m.cols[3].y; dest[14] = (float)m.cols[3].z; dest[15] = 1;
            out.ibms[ji] = mat4_invert(mat);
        } else {
            out.ibms[ji] = mat4_identity();
        }
    }

    // Animations
    for (size_t si = 0; si < scene->anim_stacks.count; ++si) {
        ufbx_anim_stack* stack = scene->anim_stacks[si];
        ufbx_bake_opts bake_opts = { 0 };
        ufbx_baked_anim* baked_anim = ufbx_bake_anim(scene, stack->anim, &bake_opts, &error);
        if (!baked_anim) continue;

        AnimationDef ad;
        ad.name = stack->name.data;
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
            float prev_q[4];
            memcpy(prev_q, out.joints[ji].rotate, 16);
            
            double duration = stack->time_end - stack->time_begin;
            int num_frames = (int)(duration * BASE_FPS) + 1;

            for (int fi = 0; fi < num_frames; ++fi) {
                double key_time = (double)fi / (double)BASE_FPS;
                ufbx_transform tr = ufbx_evaluate_transform(stack->anim, node, key_time);
                float t[3] = {(float)tr.translation.x, (float)tr.translation.y, (float)tr.translation.z};
                float q[4] = {(float)tr.rotation.x, (float)tr.rotation.y, (float)tr.rotation.z, (float)tr.rotation.w};
                float s[3] = {(float)tr.scale.x, (float)tr.scale.y, (float)tr.scale.z};
                
                stabilize_trs(t, q, s);
                float dot = q[0]*prev_q[0] + q[1]*prev_q[1] + q[2]*prev_q[2] + q[3]*prev_q[3];
                if (dot < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
                memcpy(prev_q, q, 16);
                
                ba.translation.times.push_back(key_time);
                ba.translation.values.push_back(t[0]);
                ba.translation.values.push_back(t[1]);
                ba.translation.values.push_back(t[2]);
                ba.rotation.times.push_back(key_time);
                ba.rotation.values.push_back(q[0]);
                ba.rotation.values.push_back(q[1]);
                ba.rotation.values.push_back(q[2]);
                ba.rotation.values.push_back(q[3]);
                ba.scale.times.push_back(key_time);
                ba.scale.values.push_back(s[0]);
                ba.scale.values.push_back(s[1]);
                ba.scale.values.push_back(s[2]);
            }
        }
        out.animations.push_back(ad);
        ufbx_free_baked_anim(baked_anim);
    }

    ufbx_free_scene(scene);
    return true;
}
