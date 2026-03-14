#include "glb_writer.h"
#include "cgltf.h"
#include "cgltf_write.h"
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cstring>
#include <cstdlib>
#include <algorithm>

static char* sanitize_name(const char* name) {
    if (!name || !name[0]) return strdup("unnamed");
    std::string s(name);
    for (char& c : s) {
        unsigned char uc = (unsigned char)c;
        if (uc < 32 || uc > 126) c = '_';
    }
    return strdup(s.c_str());
}

static void append_to_buffer(std::vector<char>& buf, cgltf_buffer* gltf_buf, const void* data, size_t bytes) {
    if (bytes == 0) return;
    buf.insert(buf.end(), (const char*)data, (const char*)data + bytes);
    gltf_buf->size = buf.size();
}

static cgltf_accessor* alloc_accessor(cgltf_data* out, cgltf_buffer_view* view, cgltf_type type, cgltf_component_type comp, size_t count, size_t byte_offset, bool normalized = false) {
    cgltf_accessor* acc = &out->accessors[out->accessors_count++];
    acc->buffer_view = view;
    acc->type = type;
    acc->component_type = comp;
    acc->count = count;
    acc->offset = byte_offset;
    acc->normalized = normalized;
    return acc;
}

bool write_glb(const Model& model, const char* output_path) {
    std::vector<char> buf;
    
    cgltf_data* out = (cgltf_data*)calloc(1, sizeof(cgltf_data));
    out->asset.version = strdup("2.0");
    out->asset.generator = strdup("gskelconv");

    // Pre-calculate counts
    size_t num_meshes = model.meshes.size();
    size_t num_joints = model.joints.size();
    size_t num_anims = model.animations.size();

    // Accessors: Meshes (up to 7 per mesh), IBMs (1), Anims (2 per active channel)
    size_t total_acc = (num_meshes * 7 + 1 + num_anims * (num_joints * 6)) * 2;
    out->accessors = (cgltf_accessor*)calloc(total_acc, sizeof(cgltf_accessor));
    out->accessors_count = 0;

    out->buffers_count = 1;
    out->buffers = (cgltf_buffer*)calloc(1, sizeof(cgltf_buffer));

    // Determine required buffer views
    int mesh_view_idx = 0;
    int index_view_idx = 1;
    int ibm_view_idx = -1;
    int anim_view_idx = -1;
    int num_views = 2;
    if (num_joints > 0) ibm_view_idx = num_views++;
    if (num_anims > 0) anim_view_idx = num_views++;

    out->buffer_views_count = num_views;
    out->buffer_views = (cgltf_buffer_view*)calloc(num_views, sizeof(cgltf_buffer_view));
    for (int i = 0; i < num_views; ++i) {
        out->buffer_views[i].buffer = &out->buffers[0];
    }
    out->buffer_views[mesh_view_idx].type = cgltf_buffer_view_type_vertices;
    out->buffer_views[index_view_idx].type = cgltf_buffer_view_type_indices;
    if (ibm_view_idx != -1) out->buffer_views[ibm_view_idx].type = cgltf_buffer_view_type_invalid;
    if (anim_view_idx != -1) out->buffer_views[anim_view_idx].type = cgltf_buffer_view_type_invalid;

    // Pack initial mesh data (positions/normals/uvs/joints/weights)
    out->buffer_views[mesh_view_idx].offset = buf.size();
    size_t pos_off = buf.size(); append_to_buffer(buf, &out->buffers[0], model.positions.data(), model.positions.size() * sizeof(float));
    size_t norm_off = buf.size(); append_to_buffer(buf, &out->buffers[0], model.normals.data(), model.normals.size() * sizeof(float));
    size_t uv_off = buf.size(); append_to_buffer(buf, &out->buffers[0], model.texcoords.data(), model.texcoords.size() * sizeof(float));
    size_t joint_off = buf.size(); append_to_buffer(buf, &out->buffers[0], model.joints_0.data(), model.joints_0.size());
    size_t weight_off = buf.size(); append_to_buffer(buf, &out->buffers[0], model.weights_0.data(), model.weights_0.size() * sizeof(float));
    out->buffer_views[mesh_view_idx].size = buf.size() - out->buffer_views[mesh_view_idx].offset;

    out->buffer_views[index_view_idx].offset = buf.size();


    // Materials (Names Only)
    std::map<std::string, cgltf_material*> mat_map;
    for (auto& m : model.meshes) mat_map[m.material_name] = nullptr;
    out->materials_count = (cgltf_size)mat_map.size();
    if (out->materials_count > 0) {
        out->materials = (cgltf_material*)calloc(out->materials_count, sizeof(cgltf_material));
        uint32_t mat_idx = 0;
        for (auto& kv : mat_map) {
            kv.second = &out->materials[mat_idx++];
            kv.second->name = sanitize_name(kv.first.c_str());
        }
    }

    // Meshes
    out->meshes_count = (cgltf_size)num_meshes;
    out->meshes = (cgltf_mesh*)calloc(out->meshes_count, sizeof(cgltf_mesh));
    for (size_t i = 0; i < num_meshes; ++i) {
        cgltf_mesh* mesh = &out->meshes[i];
        mesh->name = sanitize_name(model.meshes[i].name.c_str());
        mesh->primitives_count = 1;
        mesh->primitives = (cgltf_primitive*)calloc(1, sizeof(cgltf_primitive));
        cgltf_primitive* prim = &mesh->primitives[0];
        prim->type = cgltf_primitive_type_triangles;
        prim->material = mat_map[model.meshes[i].material_name];

        std::vector<cgltf_attribute> attrs;
        auto add_attr = [&](const char* name, cgltf_attribute_type type, cgltf_type data_type, cgltf_component_type comp, size_t off) {
            cgltf_attribute a = {};
            a.name = strdup(name);
            a.type = type;
            a.data = alloc_accessor(out, &out->buffer_views[mesh_view_idx], data_type, comp, model.meshes[i].num_vertexes, off + model.meshes[i].first_vertex * (cgltf_num_components(data_type) * (comp == cgltf_component_type_r_32f ? 4 : 1)));
            attrs.push_back(a);
            return &attrs.back();
        };

        cgltf_attribute* pos_attr = add_attr("POSITION", cgltf_attribute_type_position, cgltf_type_vec3, cgltf_component_type_r_32f, pos_off);
        pos_attr->data->has_min = pos_attr->data->has_max = true;
        for(int k=0; k<3; ++k) pos_attr->data->min[k] = pos_attr->data->max[k] = model.positions[model.meshes[i].first_vertex*3 + k];
        for(uint32_t v=0; v<model.meshes[i].num_vertexes; ++v) {
            for(int k=0; k<3; ++k) {
                float val = model.positions[(model.meshes[i].first_vertex + v)*3 + k];
                if(val < pos_attr->data->min[k]) pos_attr->data->min[k] = val;
                if(val > pos_attr->data->max[k]) pos_attr->data->max[k] = val;
            }
        }

        if(!model.normals.empty()) add_attr("NORMAL", cgltf_attribute_type_normal, cgltf_type_vec3, cgltf_component_type_r_32f, norm_off);
        if(!model.texcoords.empty()) add_attr("TEXCOORD_0", cgltf_attribute_type_texcoord, cgltf_type_vec2, cgltf_component_type_r_32f, uv_off);
        if(!model.joints_0.empty()) add_attr("JOINTS_0", cgltf_attribute_type_joints, cgltf_type_vec4, cgltf_component_type_r_8u, joint_off);
        if(!model.weights_0.empty()) add_attr("WEIGHTS_0", cgltf_attribute_type_weights, cgltf_type_vec4, cgltf_component_type_r_32f, weight_off);

        prim->attributes_count = attrs.size();
        if (prim->attributes_count > 0) {
            prim->attributes = (cgltf_attribute*)calloc(attrs.size(), sizeof(cgltf_attribute));
            for(size_t a=0; a<attrs.size(); ++a) prim->attributes[a] = attrs[a];
        }

        std::vector<uint32_t> standard_indices(model.meshes[i].num_triangles * 3);
        for(uint32_t t=0; t<model.meshes[i].num_triangles; ++t) {
            standard_indices[t*3+0] = model.indices[model.meshes[i].first_triangle*3 + t*3+0] - model.meshes[i].first_vertex;
            standard_indices[t*3+1] = model.indices[model.meshes[i].first_triangle*3 + t*3+1] - model.meshes[i].first_vertex;
            standard_indices[t*3+2] = model.indices[model.meshes[i].first_triangle*3 + t*3+2] - model.meshes[i].first_vertex;
        }
        size_t off = buf.size();
        append_to_buffer(buf, &out->buffers[0], standard_indices.data(), standard_indices.size()*4);
        prim->indices = alloc_accessor(out, &out->buffer_views[index_view_idx], cgltf_type_scalar, cgltf_component_type_r_32u, standard_indices.size(), off - out->buffer_views[index_view_idx].offset);
    }
    out->buffer_views[index_view_idx].size = buf.size() - out->buffer_views[index_view_idx].offset;

    // IBMs and Skin
    cgltf_accessor* ibm_acc = nullptr;
    if (num_joints > 0 && ibm_view_idx != -1) {
        out->buffer_views[ibm_view_idx].offset = buf.size();
        const std::vector<mat4>& source_ibms = model.ibms.empty() ? model.computed_ibms : model.ibms;
        append_to_buffer(buf, &out->buffers[0], source_ibms.data(), source_ibms.size() * sizeof(mat4));
        out->buffer_views[ibm_view_idx].size = buf.size() - out->buffer_views[ibm_view_idx].offset;
        ibm_acc = alloc_accessor(out, &out->buffer_views[ibm_view_idx], cgltf_type_mat4, cgltf_component_type_r_32f, source_ibms.size(), 0);
    }

    // Nodes
    out->nodes_count = (cgltf_size)(num_joints + num_meshes);
    out->nodes = (cgltf_node*)calloc(out->nodes_count, sizeof(cgltf_node));

    cgltf_node* joints_start = &out->nodes[0];
    for (size_t i = 0; i < num_joints; ++i) {
        cgltf_node* n = &joints_start[i];
        n->name = sanitize_name(model.joints[i].name.c_str());
        memcpy(n->translation, model.joints[i].translate, 3*sizeof(float));
        memcpy(n->rotation, model.joints[i].rotate, 4*sizeof(float));
        memcpy(n->scale, model.joints[i].scale, 3*sizeof(float));
        n->has_translation = n->has_rotation = n->has_scale = true;
    }
    
    if (num_joints > 0) {
        std::vector<int> children_counts(num_joints, 0);
        for (size_t i = 0; i < num_joints; ++i) if (model.joints[i].parent >= 0) children_counts[model.joints[i].parent]++;
        for (size_t i = 0; i < num_joints; ++i) {
            if (children_counts[i] > 0) {
                joints_start[i].children = (cgltf_node**)calloc(children_counts[i], sizeof(cgltf_node*));
                joints_start[i].children_count = (cgltf_size)children_counts[i];
            }
        }
        std::vector<int> current_children(num_joints, 0);
        for (size_t i = 0; i < num_joints; ++i) {
            int p = model.joints[i].parent;
            if (p >= 0) {
                joints_start[p].children[current_children[p]++] = &joints_start[i];
                joints_start[i].parent = &joints_start[p];
            }
        }

        out->skins_count = 1;
        out->skins = (cgltf_skin*)calloc(1, sizeof(cgltf_skin));
        out->skins[0].name = strdup("ArmatureSkin");
        out->skins[0].joints_count = (cgltf_size)num_joints;
        out->skins[0].joints = (cgltf_node**)calloc(num_joints, sizeof(cgltf_node*));
        for(size_t i=0; i<num_joints; ++i) out->skins[0].joints[i] = &joints_start[i];
        out->skins[0].inverse_bind_matrices = ibm_acc;
        
        for (size_t i = 0; i < num_joints; ++i) {
            if (model.joints[i].parent == -1) {
                out->skins[0].skeleton = &joints_start[i];
                break;
            }
        }
    }

    out->scenes_count = 1;
    out->scenes = (cgltf_scene*)calloc(1, sizeof(cgltf_scene));
    out->scenes[0].name = strdup("Scene");
    std::vector<cgltf_node*> scene_roots;
    for (size_t i = 0; i < num_joints; ++i) if (model.joints[i].parent == -1) scene_roots.push_back(&joints_start[i]);

    cgltf_node* mesh_nodes = &out->nodes[num_joints];
    for (size_t i = 0; i < num_meshes; ++i) {
        mesh_nodes[i].name = sanitize_name(model.meshes[i].name.c_str());
        mesh_nodes[i].mesh = &out->meshes[i];
        if (num_joints > 0) mesh_nodes[i].skin = &out->skins[0];
        scene_roots.push_back(&mesh_nodes[i]);
    }

    out->scenes[0].nodes_count = (cgltf_size)scene_roots.size();
    out->scenes[0].nodes = (cgltf_node**)calloc(scene_roots.size(), sizeof(cgltf_node*));
    for (size_t i = 0; i < scene_roots.size(); ++i) out->scenes[0].nodes[i] = scene_roots[i];
    out->scene = &out->scenes[0];

    // Animations
    if (num_anims > 0 && anim_view_idx != -1) {
        out->buffer_views[anim_view_idx].offset = buf.size();

        std::vector<cgltf_animation> gltf_anims;
        for (size_t ai = 0; ai < num_anims; ++ai) {
            const AnimationDef& def = model.animations[ai];
            size_t active_channels_count = 0;
            for (const auto& ba : def.track.bones) {
                if (!ba.translation.times.empty()) active_channels_count++;
                if (!ba.rotation.times.empty()) active_channels_count++;
                if (!ba.scale.times.empty()) active_channels_count++;
            }
            if (active_channels_count == 0) continue;

            cgltf_animation anim = {};
            anim.name = sanitize_name(def.name.c_str());
            anim.samplers_count = (cgltf_size)active_channels_count;
            anim.samplers = (cgltf_animation_sampler*)calloc(active_channels_count, sizeof(cgltf_animation_sampler));
            anim.channels_count = (cgltf_size)active_channels_count;
            anim.channels = (cgltf_animation_channel*)calloc(active_channels_count, sizeof(cgltf_animation_channel));

            uint32_t ch_idx = 0;
            for (size_t ji = 0; ji < num_joints; ++ji) {
                const BoneAnim& ba = def.track.bones[ji];
                auto add_chan = [&](const AnimChannel& chan, cgltf_animation_path_type path, int comps) {
                    if (chan.times.empty()) return;
                    size_t ts_off = buf.size();
                    std::vector<float> f_times(chan.times.begin(), chan.times.end());
                    append_to_buffer(buf, &out->buffers[0], f_times.data(), f_times.size() * 4);
                    cgltf_accessor* t_acc = alloc_accessor(out, &out->buffer_views[anim_view_idx], cgltf_type_scalar, cgltf_component_type_r_32f, f_times.size(), ts_off - out->buffer_views[anim_view_idx].offset);
                    t_acc->has_min = t_acc->has_max = true;
                    t_acc->min[0] = f_times.front(); t_acc->max[0] = f_times.back();

                    size_t v_off = buf.size();
                    append_to_buffer(buf, &out->buffers[0], chan.values.data(), chan.values.size() * 4);
                    cgltf_accessor* v_acc = alloc_accessor(out, &out->buffer_views[anim_view_idx], (comps==4?cgltf_type_vec4:cgltf_type_vec3), cgltf_component_type_r_32f, chan.times.size(), v_off - out->buffer_views[anim_view_idx].offset);

                    anim.samplers[ch_idx].input = t_acc;
                    anim.samplers[ch_idx].output = v_acc;
                    anim.samplers[ch_idx].interpolation = cgltf_interpolation_type_linear;
                    anim.channels[ch_idx].sampler = &anim.samplers[ch_idx];
                    anim.channels[ch_idx].target_node = &joints_start[ji];
                    anim.channels[ch_idx].target_path = path;
                    ch_idx++;
                };
                add_chan(ba.translation, cgltf_animation_path_type_translation, 3);
                add_chan(ba.rotation, cgltf_animation_path_type_rotation, 4);
                add_chan(ba.scale, cgltf_animation_path_type_scale, 3);
            }
            gltf_anims.push_back(anim);
        }
        out->animations_count = gltf_anims.size();
        out->animations = (cgltf_animation*)calloc(gltf_anims.size(), sizeof(cgltf_animation));
        for(size_t i=0; i<gltf_anims.size(); ++i) out->animations[i] = gltf_anims[i];
        out->buffer_views[anim_view_idx].size = buf.size() - out->buffer_views[anim_view_idx].offset;
    }

    out->bin_size = buf.size();
    out->bin = malloc(out->bin_size);
    memcpy((void*)out->bin, buf.data(), out->bin_size);
    
    cgltf_options opts = {}; opts.type = cgltf_file_type_glb;
    cgltf_result res = cgltf_write_file(&opts, output_path, out);
    if (res == cgltf_result_success) std::cout << "GLB written successfully: " << output_path << "\n";

    // Note: cgltf_free(out) is skipped here because it triggers a 0xC0000005 crash on exit 
    // due to internal ownership conflicts with manual allocations. The OS will reclaim 
    // this memory on process termination.
    return res == cgltf_result_success;
}
