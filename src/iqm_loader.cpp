#include "iqm_loader.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>
#include "anim_cfg.h"

bool load_iqm(const char* path, Model& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    f.seekg(0, std::ios::end);
    size_t file_size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> buffer(file_size);
    f.read(buffer.data(), file_size);
    f.close();

    // Check for animation.cfg alongside the file
    std::string cfg_path = find_animation_cfg(path);
    std::vector<gs_legacy_framegroup> overrides;
    std::vector<AnimConfigEntry> entries;
    
    if (!cfg_path.empty()) {
        entries = parse_animation_cfg(cfg_path);
        for (const auto& entry : entries) {
            gs_legacy_framegroup fg;
            fg.name = entry.name.c_str(); 
            fg.first_frame = entry.first_frame;
            fg.num_frames = entry.last_frame - entry.first_frame + 1;
            fg.fps = entry.fps;
            overrides.push_back(fg);
        }
    }

    // Pass the overrides to the memory loader
    return load_iqm_from_memory(buffer.data(), file_size, out, 
                                overrides.empty() ? nullptr : overrides.data(), 
                                (uint32_t)overrides.size());
}

bool load_iqm_from_memory(const void* data, size_t size, Model& out, const gs_legacy_framegroup* external_anims, uint32_t num_external_anims) {
    if (!data || size < sizeof(iqmheader)) return false;

    const char* buffer = (const char*)data;
    const iqmheader* hdr = (const iqmheader*)buffer;
    if (std::memcmp(hdr->magic, IQM_MAGIC, 16) != 0) return false;

    const char* text_pool = buffer + hdr->ofs_text;

    // Joints
    out.joints.resize(hdr->num_joints);
    if (hdr->num_joints > 0) {
        const iqmjoint* iqm_joints = (const iqmjoint*)(buffer + hdr->ofs_joints);
        for (uint32_t i = 0; i < hdr->num_joints; ++i) {
            out.joints[i].name = text_pool + iqm_joints[i].name;
            out.joints[i].parent = iqm_joints[i].parent;

            memcpy(out.joints[i].translate, iqm_joints[i].translate, 12);
            memcpy(out.joints[i].rotate, iqm_joints[i].rotate, 16);
            memcpy(out.joints[i].scale, iqm_joints[i].scale, 12);
        }
        // Compute and store IBMs as the "source of truth" for the mesh
        out.compute_bind_pose();
        out.ibms = out.computed_ibms;
    }

    // Meshes and Materials
    if (hdr->num_meshes > 0) {
        out.meshes.resize(hdr->num_meshes);
        const iqmmesh* iqm_meshes = (const iqmmesh*)(buffer + hdr->ofs_meshes);
        for (uint32_t i = 0; i < hdr->num_meshes; ++i) {
            out.meshes[i].name = text_pool + iqm_meshes[i].name;
            std::string mat_name = text_pool + iqm_meshes[i].material;

            int mat_idx = -1;
            for (int m = 0; m < (int)out.materials.size(); ++m) {
                if (out.materials[m].name == mat_name) {
                    mat_idx = m;
                    break;
                }
            }

            if (mat_idx == -1) {
                mat_idx = (int)out.materials.size();
                Material mat;
                mat.name = mat_name;
                mat.color_map = mat_name;
                mat.material_type = (is_pbr_suffix(mat_name) || !mat.metallic_map.empty() || !mat.roughness_map.empty()) ? 0 : 1;
                out.materials.push_back(mat);
            }

            out.meshes[i].material_idx = mat_idx;
            out.meshes[i].first_vertex = iqm_meshes[i].first_vertex;
            out.meshes[i].num_vertexes = iqm_meshes[i].num_vertexes;
            out.meshes[i].first_triangle = iqm_meshes[i].first_triangle;
            out.meshes[i].num_triangles = iqm_meshes[i].num_triangles;
        }
    }

    // Vertex data
    const iqmvertexarray* va = (const iqmvertexarray*)(buffer + hdr->ofs_vertexarrays);
    for (uint32_t i = 0; i < hdr->num_vertexarrays; ++i) {
        const char* src = buffer + va[i].offset;
        if (va[i].type == IQM_POSITION && va[i].format == IQM_FLOAT && va[i].size == 3) {
            out.positions.resize(hdr->num_vertexes * 3);
            memcpy(out.positions.data(), src, hdr->num_vertexes * 3 * sizeof(float));
        } else if (va[i].type == IQM_NORMAL && va[i].format == IQM_FLOAT && va[i].size == 3) {
            out.normals.resize(hdr->num_vertexes * 3);
            memcpy(out.normals.data(), src, hdr->num_vertexes * 3 * sizeof(float));
        } else if (va[i].type == IQM_TANGENT && va[i].format == IQM_FLOAT && va[i].size == 4) {
            out.tangents.resize(hdr->num_vertexes * 4);
            memcpy(out.tangents.data(), src, hdr->num_vertexes * 4 * sizeof(float));
        } else if (va[i].type == IQM_COLOR && va[i].format == IQM_UBYTE && va[i].size == 4) {
            out.colors.resize(hdr->num_vertexes * 4);
            const uint8_t* c_src = (const uint8_t*)src;
            for (uint32_t v = 0; v < hdr->num_vertexes * 4; ++v) out.colors[v] = c_src[v] / 255.0f;
        } else if (va[i].type == IQM_TEXCOORD && va[i].format == IQM_FLOAT && va[i].size == 2) {
            out.texcoords.resize(hdr->num_vertexes * 2);
            memcpy(out.texcoords.data(), src, hdr->num_vertexes * 2 * sizeof(float));
        } else if (va[i].type == IQM_BLENDINDICES && va[i].format == IQM_UBYTE && va[i].size == 4) {
            out.joints_0.resize(hdr->num_vertexes * 4);
            memcpy(out.joints_0.data(), src, hdr->num_vertexes * 4);
        } else if (va[i].type == IQM_BLENDWEIGHTS && va[i].size == 4) {
            out.weights_0.resize(hdr->num_vertexes * 4);
            if (va[i].format == IQM_UBYTE) {
                const uint8_t* u8src = (const uint8_t*)src;
                for (uint32_t v = 0; v < hdr->num_vertexes * 4; ++v) out.weights_0[v] = u8src[v] / 255.0f;
            } else if (va[i].format == IQM_FLOAT) {
                memcpy(out.weights_0.data(), src, hdr->num_vertexes * 4 * sizeof(float));
            }
        }
    }

    // Indices
    out.indices.resize(hdr->num_triangles * 3);
    if (hdr->num_triangles > 0) {
        const uint32_t* iqm_indices = (const uint32_t*)(buffer + hdr->ofs_triangles);
        memcpy(out.indices.data(), iqm_indices, hdr->num_triangles * 3 * sizeof(uint32_t));
    }

    // Animations (Unpack into sparse tracks)
    if (hdr->num_frames > 0) {
        std::vector<iqmpose> orig_poses(hdr->num_poses);
        if (hdr->num_poses > 0) memcpy(orig_poses.data(), buffer + hdr->ofs_poses, hdr->num_poses * sizeof(iqmpose));

        uint32_t orig_framechannels = 0;
        for (uint32_t p = 0; p < hdr->num_poses; ++p) {
            for (int c = 0; c < 10; ++c) if (orig_poses[p].mask & (1 << c)) orig_framechannels++;
        }

        const unsigned short* orig_fdata = (const unsigned short*)(buffer + hdr->ofs_frames);

        std::vector<float> dense_frames(hdr->num_frames * hdr->num_poses * 10);
        for (uint32_t f = 0; f < hdr->num_frames; ++f) {
            const unsigned short* fin = orig_fdata + f * orig_framechannels;
            float* fout = dense_frames.data() + f * hdr->num_poses * 10;
            uint32_t ch_idx = 0;
            for (size_t p = 0; p < hdr->num_poses; ++p) {
                const iqmpose& op = orig_poses[p];
                float vals[10];
                for (int c = 0; c < 10; ++c) {
                    vals[c] = op.channeloffset[c];
                    if (op.mask & (1 << c)) vals[c] += fin[ch_idx++] * op.channelscale[c];
                }
                memcpy(fout + p * 10, vals, 40);
            }
        }

        // Collect animation definitions
        struct TempAnim { std::string name; int first, count; float fps; };
        std::vector<TempAnim> anims;

        if (external_anims && num_external_anims > 0) {
            uint32_t n = (num_external_anims > 1024) ? 1024 : num_external_anims;
            for (uint32_t i = 0; i < n; ++i) {
                int first = external_anims[i].first_frame;
                int count = external_anims[i].num_frames;
                if (first >= (int)hdr->num_frames) continue;
                if (first < 0) first = 0;
                if (first + count > (int)hdr->num_frames) {
                    count = (int)hdr->num_frames - first;
                }
                if (count <= 0) continue;
                anims.push_back({ external_anims[i].name ? external_anims[i].name : "unnamed", first, count, external_anims[i].fps });
            }
        } else if (hdr->num_anims > 0) {
            const iqmanim* iqm_anims = (const iqmanim*)(buffer + hdr->ofs_anims);
            for (uint32_t i = 0; i < hdr->num_anims; ++i) {
                anims.push_back({ text_pool + iqm_anims[i].name, iqm_anims[i].first_frame, iqm_anims[i].num_frames, (iqm_anims[i].framerate > 0.0f) ? iqm_anims[i].framerate : BASE_FPS });
            }
        } else {
            anims.push_back({ "base", 0, (int)hdr->num_frames, BASE_FPS });
        }

        for (const auto& a : anims) {
            AnimationDef ad;
            ad.name = a.name;
            ad.duration = (a.count > 0) ? (double)(a.count - 1) / a.fps : 0.0;
            ad.bones.resize(out.joints.size());
            for (int f = a.first; f < a.first + a.count; ++f) {
                if (f < 0 || f >= (int)hdr->num_frames) continue;
                double time = (double)(f - a.first) / a.fps;
                float* fptr = dense_frames.data() + (size_t)f * hdr->num_poses * 10;

                for (size_t ji = 0; ji < out.joints.size(); ++ji) {
                    BoneAnim& ba = ad.bones[ji];
                    float* bptr = fptr + ji * 10;
                    ba.translation.times.push_back(time);
                    ba.translation.values.push_back(bptr[0]);
                    ba.translation.values.push_back(bptr[1]);
                    ba.translation.values.push_back(bptr[2]);
                    ba.rotation.times.push_back(time);
                    ba.rotation.values.push_back(bptr[3]);
                    ba.rotation.values.push_back(bptr[4]);
                    ba.rotation.values.push_back(bptr[5]);
                    ba.rotation.values.push_back(bptr[6]);
                    ba.scale.times.push_back(time);
                    ba.scale.values.push_back(bptr[7]);
                    ba.scale.values.push_back(bptr[8]);
                    ba.scale.values.push_back(bptr[9]);
                }
            }
            out.animations.push_back(ad);
        }
    }

    out.orientation = GS_Z_UP_RIGHTHANDED_X_FWD;
    out.winding = GS_WINDING_CW;

    return true;
}
