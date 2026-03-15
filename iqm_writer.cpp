#include "iqm_writer.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>

static uint32_t write_text(std::vector<char>& pool, const std::string& s) {
    if (s.empty()) return 0;
    uint32_t pos = (uint32_t)pool.size();
    for (char c : s) pool.push_back(c);
    pool.push_back('\0');
    return pos;
}

bool write_iqm(const Model& model, const char* output_path, bool force_single_anim) {
    std::vector<gs_legacy_framegroup> metadata;
    std::vector<uint8_t> buffer = write_iqm_to_memory(model, force_single_anim, &metadata);
    if (buffer.empty()) return false;

    std::ofstream f(output_path, std::ios::binary);
    if (!f) return false;
    f.write((const char*)buffer.data(), buffer.size());
    f.close();

    // 6. Write animation.cfg using the metadata we just calculated during baking
    if (!metadata.empty()) {
        std::string cfg_path = output_path;
        size_t dot = cfg_path.find_last_of('.');
        if (dot != std::string::npos) cfg_path = cfg_path.substr(0, dot) + ".cfg";
        else cfg_path += ".cfg";
        
        std::ofstream cfg(cfg_path);
        if (cfg) {
            for (const auto& fg : metadata) {
                int loop = 0; 
                cfg << fg.first_frame << " " << fg.first_frame + fg.num_frames - 1 << " " 
                    << loop << " " << fg.fps << " // " << fg.name << "\n";
            }
        }
    }

    std::cout << "IQM written successfully: " << output_path << "\n";
    return true;
}

std::vector<uint8_t> write_iqm_to_memory(const Model& model, bool force_single_anim, std::vector<gs_legacy_framegroup>* out_metadata) {
    std::vector<uint8_t> buffer;
    iqmheader hdr = {};
    memcpy(hdr.magic, IQM_MAGIC, 16);
    hdr.version = IQM_VERSION;

    std::vector<char> text;
    text.push_back('\0'); 

    // 1. Prepare Joints (Convert Y-up to Z-up)
    std::vector<iqmjoint> iqm_joints(model.joints.size());
    float q_rot_inv[4] = {0.7071068f, 0.0f, 0.0f, 0.7071068f}; // +90X
    
    for (size_t i = 0; i < model.joints.size(); ++i) {
        iqm_joints[i].name = write_text(text, model.joints[i].name);
        iqm_joints[i].parent = model.joints[i].parent;
        
        float t[3]; memcpy(t, model.joints[i].translate, 12);
        float r[4]; memcpy(r, model.joints[i].rotate, 16);
        quat_normalize(r);
        
        if (model.joints[i].parent == -1) {
            float t_zup[3] = {t[0], -t[2], t[1]};
            memcpy(t, t_zup, 12);
            float r_new[4];
            quat_mul(q_rot_inv, r, r_new);
            quat_normalize(r_new);
            memcpy(r, r_new, 16);
        }
        
        memcpy(iqm_joints[i].translate, t, 12);
        memcpy(iqm_joints[i].rotate, r, 16);
        memcpy(iqm_joints[i].scale, model.joints[i].scale, 12);
    }

    // 2. Prepare Meshes
    std::vector<iqmmesh> iqm_meshes(model.meshes.size());
    for (size_t i = 0; i < model.meshes.size(); ++i) {
        iqm_meshes[i].name = write_text(text, model.meshes[i].name);
        iqm_meshes[i].material = write_text(text, model.meshes[i].material_name);
        iqm_meshes[i].first_vertex = model.meshes[i].first_vertex;
        iqm_meshes[i].num_vertexes = model.meshes[i].num_vertexes;
        iqm_meshes[i].first_triangle = model.meshes[i].first_triangle;
        iqm_meshes[i].num_triangles = model.meshes[i].num_triangles;
    }

    // 3. Prepare Animations
    std::vector<iqmanim> iqm_anims;
    uint32_t total_iqm_frames = 0;

    std::vector<uint32_t> clip_frame_counts;
    for (const auto& ad : model.animations) {
        double clip_duration = 0;
        for (const auto& ba : ad.bones) {
            if (!ba.translation.times.empty()) clip_duration = std::max(clip_duration, ba.translation.times.back());
            if (!ba.rotation.times.empty()) clip_duration = std::max(clip_duration, ba.rotation.times.back());
            if (!ba.scale.times.empty()) clip_duration = std::max(clip_duration, ba.scale.times.back());
        }
        uint32_t nf = (uint32_t)std::round(clip_duration * BASE_FPS) + 1;
        if (nf == 1 && clip_duration == 0) nf = 1;
        clip_frame_counts.push_back(nf);
        total_iqm_frames += nf;
    }

    if (force_single_anim) {
        iqmanim ia = {};
        ia.name = write_text(text, "all");
        ia.first_frame = 0;
        ia.num_frames = total_iqm_frames;
        ia.framerate = BASE_FPS;
        ia.flags = 0;
        iqm_anims.push_back(ia);

        if (out_metadata) {
            uint32_t f_offset = 0;
            for (size_t i = 0; i < model.animations.size(); ++i) {
                gs_legacy_framegroup fg;
                fg.name = model.animations[i].name.c_str();
                fg.first_frame = f_offset;
                fg.num_frames = clip_frame_counts[i];
                fg.fps = BASE_FPS;
                out_metadata->push_back(fg);
                f_offset += clip_frame_counts[i];
            }
        }
    } else {
        uint32_t f_offset = 0;
        for (size_t i = 0; i < model.animations.size(); ++i) {
            iqmanim ia = {};
            ia.name = write_text(text, model.animations[i].name);
            ia.first_frame = f_offset;
            ia.num_frames = clip_frame_counts[i];
            ia.framerate = BASE_FPS;
            ia.flags = 0;
            iqm_anims.push_back(ia);

            if (out_metadata) {
                gs_legacy_framegroup fg;
                fg.name = model.animations[i].name.c_str(); 
                fg.first_frame = f_offset;
                fg.num_frames = clip_frame_counts[i];
                fg.fps = BASE_FPS;
                out_metadata->push_back(fg);
            }
            f_offset += clip_frame_counts[i];
        }
    }

    std::vector<iqmpose> iqm_poses(model.joints.size());
    for (size_t i = 0; i < model.joints.size(); ++i) {
        iqm_poses[i].parent = model.joints[i].parent;
        iqm_poses[i].mask = 0x3FF;
    }

    std::vector<unsigned short> out_frames;
    if (total_iqm_frames > 0) {
        std::vector<float> zup_frames(total_iqm_frames * model.joints.size() * 10);
        std::vector<Pose> evaluated_poses;

        uint32_t f_offset = 0;
        for (size_t ai = 0; ai < model.animations.size(); ++ai) {
            const AnimationDef& ad = model.animations[ai];
            uint32_t nf = clip_frame_counts[ai];

            double clip_duration = 0;
            for (const auto& ba : ad.bones) {
                if (!ba.translation.times.empty()) clip_duration = std::max(clip_duration, ba.translation.times.back());
                if (!ba.rotation.times.empty()) clip_duration = std::max(clip_duration, ba.rotation.times.back());
                if (!ba.scale.times.empty()) clip_duration = std::max(clip_duration, ba.scale.times.back());
            }

            for (uint32_t f = 0; f < nf; ++f) {
                double time = (nf > 1) ? (double)f * (clip_duration / (double)(nf - 1)) : 0.0;
                model.evaluate_animation((int)ai, time, evaluated_poses);

                for (size_t ji = 0; ji < model.joints.size(); ++ji) {
                    float* fout = &zup_frames[(f_offset + f) * model.joints.size() * 10 + ji * 10];
                    const Pose& p = evaluated_poses[ji];
                    float vals[10];
                    memcpy(vals, p.translate, 12);
                    memcpy(&vals[3], p.rotate, 16);
                    memcpy(&vals[7], p.scale, 12);

                    if (model.joints[ji].parent == -1) {
                        float tx = vals[0], ty = vals[1], tz = vals[2];
                        vals[0] = tx; vals[1] = -tz; vals[2] = ty;
                        float r_val[4] = {vals[3], vals[4], vals[5], vals[6]};
                        float r_new[4];
                        quat_mul(q_rot_inv, r_val, r_new);
                        quat_normalize(r_new);
                        vals[3] = r_new[0]; vals[4] = r_new[1]; vals[5] = r_new[2]; vals[6] = r_new[3];
                    }
                    memcpy(fout, vals, 40);
                }
            }
            f_offset += nf;
        }

        struct ChanStat { float min = 1e30f, max = -1e30f; };
        std::vector<std::vector<ChanStat>> stats(model.joints.size(), std::vector<ChanStat>(10));
        for (uint32_t f_idx = 0; f_idx < total_iqm_frames; ++f_idx) {
            const float* fptr = &zup_frames[f_idx * model.joints.size() * 10];
            for (size_t p = 0; p < model.joints.size(); ++p) {
                for (int c = 0; c < 10; ++c) {
                    float val = fptr[p * 10 + c];
                    if (val < stats[p][c].min) stats[p][c].min = val;
                    if (val > stats[p][c].max) stats[p][c].max = val;
                }
            }
        }
        
        out_frames.resize(total_iqm_frames * model.joints.size() * 10);
        for (size_t p = 0; p < model.joints.size(); ++p) {
            for (int c = 0; c < 10; ++c) {
                float mn = stats[p][c].min, mx = stats[p][c].max;
                iqm_poses[p].channeloffset[c] = mn;
                iqm_poses[p].channelscale[c] = (mx - mn) / 65535.0f;
            }
        }

        for (uint32_t f_idx = 0; f_idx < total_iqm_frames; ++f_idx) {
            for (size_t p = 0; p < model.joints.size(); ++p) {
                for (int c = 0; c < 10; ++c) {
                    float val = zup_frames[f_idx * model.joints.size() * 10 + p * 10 + c];
                    float off = iqm_poses[p].channeloffset[c];
                    float scl = iqm_poses[p].channelscale[c];
                    if (scl > 0) out_frames[f_idx * model.joints.size() * 10 + p * 10 + c] = (unsigned short)((val - off) / scl + 0.5f);
                    else out_frames[f_idx * model.joints.size() * 10 + p * 10 + c] = 0;
                }
            }
        }
    }

    // 4. Build binary buffer
    auto append_data = [&](const void* data, size_t size) {
        size_t current = buffer.size();
        size_t padding = (4 - (current % 4)) % 4;
        for (size_t i = 0; i < padding; ++i) buffer.push_back(0);
        size_t start = buffer.size();
        const uint8_t* p = (const uint8_t*)data;
        for (size_t i = 0; i < size; ++i) buffer.push_back(p[i]);
        return (uint32_t)start;
    };

    buffer.assign(sizeof(iqmheader), 0);

    hdr.num_text = (uint32_t)text.size();
    hdr.ofs_text = append_data(text.data(), text.size());

    hdr.num_meshes = (uint32_t)iqm_meshes.size();
    hdr.ofs_meshes = append_data(iqm_meshes.data(), iqm_meshes.size() * sizeof(iqmmesh));

    std::vector<iqmvertexarray> va;
    auto add_va = [&](uint32_t type, uint32_t format, uint32_t size, const void* data, size_t bytes) {
        if (bytes == 0) return;
        uint32_t off = append_data(data, bytes);
        va.push_back({type, 0, format, size, off});
    };

    std::vector<float> zup_positions(model.positions.size());
    for(size_t i=0; i<model.positions.size()/3; ++i) {
        zup_positions[i*3+0] = model.positions[i*3+0];
        zup_positions[i*3+1] = -model.positions[i*3+2];
        zup_positions[i*3+2] = model.positions[i*3+1];
    }
    std::vector<float> zup_normals(model.normals.size());
    for(size_t i=0; i<model.normals.size()/3; ++i) {
        zup_normals[i*3+0] = model.normals[i*3+0];
        zup_normals[i*3+1] = -model.normals[i*3+2];
        zup_normals[i*3+2] = model.normals[i*3+1];
    }

    add_va(IQM_POSITION, IQM_FLOAT, 3, zup_positions.data(), zup_positions.size() * 4);
    add_va(IQM_TEXCOORD, IQM_FLOAT, 2, model.texcoords.data(), model.texcoords.size() * 4);
    add_va(IQM_NORMAL,   IQM_FLOAT, 3, zup_normals.data(), zup_normals.size() * 4);
    add_va(IQM_BLENDINDICES, IQM_UBYTE, 4, model.joints_0.data(), model.joints_0.size());
    std::vector<uint8_t> w8(model.weights_0.size());
    for(size_t i=0; i<w8.size(); ++i) w8[i] = (uint8_t)(model.weights_0[i] * 255.0f + 0.5f);
    add_va(IQM_BLENDWEIGHTS, IQM_UBYTE, 4, w8.data(), w8.size());

    hdr.num_vertexarrays = (uint32_t)va.size();
    hdr.ofs_vertexarrays = append_data(va.data(), va.size() * sizeof(iqmvertexarray));
    hdr.num_vertexes = (uint32_t)model.positions.size() / 3;

    std::vector<uint32_t> iqm_indices(model.indices.size());
    for (uint32_t i = 0; i < model.indices.size() / 3; ++i) {
        iqm_indices[i * 3 + 0] = model.indices[i * 3 + 0];
        iqm_indices[i * 3 + 1] = model.indices[i * 3 + 2];
        iqm_indices[i * 3 + 2] = model.indices[i * 3 + 1];
    }
    hdr.num_triangles = (uint32_t)iqm_indices.size() / 3;
    hdr.ofs_triangles = append_data(iqm_indices.data(), iqm_indices.size() * 4);

    hdr.num_joints = (uint32_t)iqm_joints.size();
    hdr.ofs_joints = append_data(iqm_joints.data(), iqm_joints.size() * sizeof(iqmjoint));

    hdr.num_poses = (uint32_t)iqm_poses.size();
    hdr.ofs_poses = append_data(iqm_poses.data(), iqm_poses.size() * sizeof(iqmpose));

    hdr.num_anims = (uint32_t)iqm_anims.size();
    hdr.ofs_anims = append_data(iqm_anims.data(), iqm_anims.size() * sizeof(iqmanim));

    hdr.num_frames = total_iqm_frames;
    hdr.num_framechannels = (uint32_t)model.joints.size() * 10;
    hdr.ofs_frames = append_data(out_frames.data(), out_frames.size() * 2);

    if (model.has_bounds) {
        uint32_t nb = std::max(1u, total_iqm_frames);
        std::vector<iqmbounds> b(nb);
        for(auto& bi : b) {
            bi.bbmin[0] = model.mins[0]; bi.bbmin[1] = -model.maxs[2]; bi.bbmin[2] = model.mins[1];
            bi.bbmax[0] = model.maxs[0]; bi.bbmax[1] = -model.mins[2]; bi.bbmax[2] = model.maxs[1];
            bi.xyradius = model.xyradius; bi.radius = model.radius;
        }
        hdr.ofs_bounds = append_data(b.data(), b.size() * sizeof(iqmbounds));
    }

    hdr.filesize = (uint32_t)buffer.size();
    memcpy(buffer.data(), &hdr, sizeof(iqmheader));

    return buffer;
}
