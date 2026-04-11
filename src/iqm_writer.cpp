#include "iqm_writer.h"
#include "orientation.h"
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

bool write_iqm(Model& model, const char* output_path, bool force_single_anim) {
    std::vector<gs_legacy_framegroup> metadata;
    std::vector<uint8_t> buffer = write_iqm_to_memory(model, force_single_anim, &metadata);
    if (buffer.empty()) return false;

    std::ofstream f(output_path, std::ios::binary);
    if (!f) return false;
    f.write((const char*)buffer.data(), buffer.size());
    f.close();

    // Write animation.cfg using the metadata we just calculated during baking
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

    return true;
}

std::vector<uint8_t> write_iqm_to_memory(Model& model, bool force_single_anim, std::vector<gs_legacy_framegroup>* out_metadata) {
    // 0. Automation: Convert orientation to Z-up, CW winding directly on the provided model
    convert_orientation(model, GS_Z_UP_RIGHTHANDED, GS_WINDING_CW);
    model.compute_bounds(); // Ensure bounds match new orientation

    std::vector<uint8_t> buffer;
    iqmheader hdr = {};
    memcpy(hdr.magic, IQM_MAGIC, 16);
    hdr.version = IQM_VERSION;

    std::vector<char> text;
    text.push_back('\0'); 

    // 1. Prepare Joints
    std::vector<iqmjoint> iqm_joints(model.joints.size());
    for (size_t i = 0; i < model.joints.size(); ++i) {
        iqm_joints[i].name = write_text(text, model.joints[i].name);
        iqm_joints[i].parent = model.joints[i].parent;
        memcpy(iqm_joints[i].translate, model.joints[i].translate, 12);
        memcpy(iqm_joints[i].rotate, model.joints[i].rotate, 16);
        memcpy(iqm_joints[i].scale, model.joints[i].scale, 12);
    }
    
    // 2. Prepare Meshes
    std::vector<iqmmesh> iqm_meshes(model.meshes.size());
    for (size_t i = 0; i < model.meshes.size(); ++i) {
        iqm_meshes[i].name = write_text(text, model.meshes[i].name);

        std::string mat_name = "default";
        int midx = model.meshes[i].material_idx;
        if (midx >= 0 && midx < (int)model.materials.size()) {
            mat_name = model.materials[midx].name;
        }

        iqm_meshes[i].material = write_text(text, mat_name);
        iqm_meshes[i].first_vertex = model.meshes[i].first_vertex;
        iqm_meshes[i].num_vertexes = model.meshes[i].num_vertexes;
        iqm_meshes[i].first_triangle = model.meshes[i].first_triangle;
        iqm_meshes[i].num_triangles = model.meshes[i].num_triangles;
    }

    // 3. Prepare Animations with Duration Snapping
    std::vector<iqmanim> iqm_anims;
    uint32_t total_iqm_frames = 0;
    std::vector<float> dense_frames;

    std::vector<uint32_t> clip_frame_counts;
    for (size_t i = 0; i < model.animations.size(); ++i) {
        const auto& ad = model.animations[i];
        
        std::vector<float> clip_data;
        model.bake_animation((uint32_t)i, (float)BASE_FPS, clip_data);
        uint32_t nf_actual = (uint32_t)(clip_data.size() / (model.joints.size() * 10));

        clip_frame_counts.push_back(nf_actual);
        total_iqm_frames += nf_actual;
        dense_frames.insert(dense_frames.end(), clip_data.begin(), clip_data.end());
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
        struct ChanStat { float min = 1e30f, max = -1e30f; };
        std::vector<std::vector<ChanStat>> stats(model.joints.size(), std::vector<ChanStat>(10));
        for (uint32_t f_idx = 0; f_idx < total_iqm_frames; ++f_idx) {
            const float* fptr = &dense_frames[f_idx * model.joints.size() * 10];
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
                    float val = dense_frames[f_idx * model.joints.size() * 10 + p * 10 + c];
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

    // Positions and normals are already converted to Z-up
    add_va(IQM_POSITION, IQM_FLOAT, 3, model.positions.data(), model.positions.size() * 4);
    add_va(IQM_TEXCOORD, IQM_FLOAT, 2, model.texcoords.data(), model.texcoords.size() * 4);
    add_va(IQM_NORMAL,   IQM_FLOAT, 3, model.normals.data(), model.normals.size() * 4);
    add_va(IQM_BLENDINDICES, IQM_UBYTE, 4, model.joints_0.data(), model.joints_0.size());
    std::vector<uint8_t> w8(model.weights_0.size());
    for(size_t i=0; i<w8.size(); ++i) w8[i] = (uint8_t)(model.weights_0[i] * 255.0f + 0.5f);
    add_va(IQM_BLENDWEIGHTS, IQM_UBYTE, 4, w8.data(), w8.size());

    hdr.num_vertexarrays = (uint32_t)va.size();
    hdr.ofs_vertexarrays = append_data(va.data(), va.size() * sizeof(iqmvertexarray));
    hdr.num_vertexes = (uint32_t)model.positions.size() / 3;

    // Indices are already converted to CW winding
    hdr.num_triangles = (uint32_t)model.indices.size() / 3;
    hdr.ofs_triangles = append_data(model.indices.data(), model.indices.size() * 4);

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
            memcpy(bi.bbmin, model.mins, 12);
            memcpy(bi.bbmax, model.maxs, 12);
            bi.xyradius = model.xyradius; 
            bi.radius = model.radius;
        }
        hdr.ofs_bounds = append_data(b.data(), b.size() * sizeof(iqmbounds));
    }

    hdr.filesize = (uint32_t)buffer.size();
    memcpy(buffer.data(), &hdr, sizeof(iqmheader));

    return buffer;
}
