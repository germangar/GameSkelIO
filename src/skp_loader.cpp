#include "skp_loader.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <map>
#include "anim_cfg.h"

#define SKMHEADER                               "SKM1"
#define SKM_MAX_NAME                    64

typedef struct {
        char                    id[4];
        unsigned int    type;
        unsigned int    filesize;
        unsigned int    num_bones;
        unsigned int    num_meshes;
        unsigned int    ofs_meshes;
} dskmheader_t;

typedef struct {
        char                    shadername[SKM_MAX_NAME];
        char                    meshname[SKM_MAX_NAME];
        unsigned int    num_verts;
        unsigned int    num_tris;
        unsigned int    num_references;
        unsigned int    ofs_verts;
        unsigned int    ofs_texcoords;
        unsigned int    ofs_indices;
        unsigned int    ofs_references;
} dskmmesh_t;

typedef struct {
        float                   origin[3];
        float                   influence;
        float                   normal[3];
        unsigned int    bonenum;
} dskmbonevert_t;

typedef struct {
        float   st[2];
} dskmcoord_t;

typedef struct {
        char                    id[4];
        unsigned int    type;
        unsigned int    filesize;
        unsigned int    num_bones;
        unsigned int    num_frames;
        unsigned int    ofs_bones;
        unsigned int    ofs_frames;
} dskpheader_t;

typedef struct {
        char                    name[SKM_MAX_NAME];
        signed int              parent;
        unsigned int    flags;
} dskpbone_t;

typedef struct {
        float                   quat[4];
        float                   origin[3];
} dskpbonepose_t;

typedef struct {
        char                    name[SKM_MAX_NAME];
        unsigned int    ofs_bonepositions;
} dskpframe_t;

static void transform_vec(const mat4& m, const float* in, float* out, float weight) {
    out[0] += (m.m[0] * in[0] + m.m[4] * in[1] + m.m[8] * in[2] + m.m[12] * weight);
    out[1] += (m.m[1] * in[0] + m.m[5] * in[1] + m.m[9] * in[2] + m.m[13] * weight);
    out[2] += (m.m[2] * in[0] + m.m[6] * in[1] + m.m[10] * in[2] + m.m[14] * weight);
}

static void rotate_vec(const mat4& m, const float* in, float* out) {
    out[0] += (m.m[0] * in[0] + m.m[4] * in[1] + m.m[8] * in[2]);
    out[1] += (m.m[1] * in[0] + m.m[5] * in[1] + m.m[9] * in[2]);
    out[2] += (m.m[2] * in[0] + m.m[6] * in[1] + m.m[10] * in[2]);
}

static std::string read_fixed_string(const char* s, size_t max_len) {        
    size_t len = 0;
    while (len < max_len && s[len] != '\0') len++;
    return std::string(s, len);
}

bool load_skm(const char* path, Model& out) {
    std::string base_path = path;
    size_t dot = base_path.find_last_of('.');
    if (dot != std::string::npos && (base_path.substr(dot) == ".skm" || base_path.substr(dot) == ".skp")) {
        base_path = base_path.substr(0, dot);
    }

    std::string skm_path = base_path + ".skm";
    std::string skp_path = base_path + ".skp";

    std::ifstream f_skp(skp_path, std::ios::binary);
    if (!f_skp) return false;
    f_skp.seekg(0, std::ios::end);
    size_t skp_size = f_skp.tellg();
    f_skp.seekg(0, std::ios::beg);
    std::vector<char> skp_buf(skp_size);
    f_skp.read(skp_buf.data(), skp_size);
    f_skp.close();

    std::ifstream f_skm(skm_path, std::ios::binary);
    if (!f_skm) return false;
    f_skm.seekg(0, std::ios::end);
    size_t skm_size = f_skm.tellg();
    f_skm.seekg(0, std::ios::beg);
    std::vector<char> skm_buf(skm_size);
    f_skm.read(skm_buf.data(), skm_size);
    f_skm.close();

    // Check for animation.cfg alongside the file
    std::string cfg_path = find_animation_cfg(skm_path);
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

    return load_skm_from_memory(skm_buf.data(), skm_size, skp_buf.data(), skp_size, out,
                                overrides.empty() ? nullptr : overrides.data(),
                                (uint32_t)overrides.size());
}

bool load_skm_from_memory(const void* skm_data, size_t skm_size, const void* skp_data, size_t skp_size, Model& out, const gs_legacy_framegroup* external_anims, uint32_t num_external_anims) {
    if (!skm_data || !skp_data) return false;

    const char* skp_buf = (const char*)skp_data;
    const char* skm_buf = (const char*)skm_data;

    dskpheader_t* skp_hdr = (dskpheader_t*)skp_buf;
    if (std::memcmp(skp_hdr->id, "SKP1", 4) != 0 && std::memcmp(skp_hdr->id, "SKM1", 4) != 0) {
        printf("SKM Load Error: SKP magic mismatch: %.4s\n", skp_hdr->id);   
        return false;
    }

    uint32_t num_bones = skp_hdr->num_bones;
    uint32_t num_frames = skp_hdr->num_frames;

    out.joints.resize(num_bones);
    dskpbone_t* skp_bones = (dskpbone_t*)(skp_buf + skp_hdr->ofs_bones);     
    for (uint32_t i = 0; i < num_bones; ++i) {
        out.joints[i].name = read_fixed_string(skp_bones[i].name, SKM_MAX_NAME);
        out.joints[i].parent = skp_bones[i].parent;
    }

    std::vector<std::vector<dskpbonepose_t>> frame_poses(num_frames, std::vector<dskpbonepose_t>(num_bones));
    dskpframe_t* skp_frames = (dskpframe_t*)(skp_buf + skp_hdr->ofs_frames); 
    for (uint32_t f = 0; f < num_frames; ++f) {
        dskpbonepose_t* poses = (dskpbonepose_t*)(skp_buf + skp_frames[f].ofs_bonepositions);
        std::memcpy(frame_poses[f].data(), poses, num_bones * sizeof(dskpbonepose_t));
    }

    std::vector<mat4> zup_world_matrices(num_bones);
    for (uint32_t i = 0; i < num_bones; ++i) {
        float scale[3] = {1, 1, 1};
        mat4 local = mat4_from_trs(frame_poses[0][i].origin, frame_poses[0][i].quat, scale);
        if (out.joints[i].parent >= 0) {
            zup_world_matrices[i] = mat4_mul(zup_world_matrices[out.joints[i].parent], local);
        } else {
            zup_world_matrices[i] = local;
        }
    }

    for (uint32_t i = 0; i < num_bones; ++i) {
        float t[3]; std::memcpy(t, frame_poses[0][i].origin, 12);
        float r[4]; std::memcpy(r, frame_poses[0][i].quat, 16);
        float s[3] = {1, 1, 1};

        if (out.joints[i].parent == -1) {
            quat_normalize(r);
        }
        std::memcpy(out.joints[i].translate, t, 12);
        std::memcpy(out.joints[i].rotate, r, 16);
        std::memcpy(out.joints[i].scale, s, 12);
    }
    // Compute and store IBMs as the "source of truth" for the mesh
    out.compute_bind_pose();
    out.ibms = out.computed_ibms;

    dskmheader_t* skm_hdr = (dskmheader_t*)skm_buf;
    if (std::memcmp(skm_hdr->id, "SKM1", 4) != 0) {
        printf("SKM Load Error: SKM magic mismatch: %.4s\n", skm_hdr->id);   
        return false;
    }

    dskmmesh_t* skm_meshes = (dskmmesh_t*)(skm_buf + skm_hdr->ofs_meshes);   
    out.meshes.resize(skm_hdr->num_meshes);
    std::map<std::string, int> mat_name_to_idx;

    for (uint32_t m = 0; m < skm_hdr->num_meshes; ++m) {
        Mesh& mesh = out.meshes[m];
        mesh.name = read_fixed_string(skm_meshes[m].meshname, SKM_MAX_NAME); 
        std::string mat_name = read_fixed_string(skm_meshes[m].shadername, SKM_MAX_NAME);

        if (mat_name_to_idx.find(mat_name) == mat_name_to_idx.end()) {       
            Material mat;
            mat.name = mat_name;
            mat.material_type = 1; // Legacy

            mat.color_map = mat_name;
            mat.material_type = (is_pbr_suffix(mat.name) || !mat.metallic_map.empty() || !mat.roughness_map.empty()) ? 0 : 1;

            mat_name_to_idx[mat_name] = (int)out.materials.size();
            out.materials.push_back(mat);
        }

        mesh.material_idx = mat_name_to_idx[mat_name];
        mesh.first_vertex = out.positions.size() / 3;
        mesh.num_vertexes = skm_meshes[m].num_verts;
        mesh.first_triangle = out.indices.size() / 3;
        mesh.num_triangles = skm_meshes[m].num_tris;

        out.positions.resize((mesh.first_vertex + mesh.num_vertexes) * 3);   
        out.normals.resize((mesh.first_vertex + mesh.num_vertexes) * 3);     
        out.texcoords.resize((mesh.first_vertex + mesh.num_vertexes) * 2);   
        out.joints_0.resize((mesh.first_vertex + mesh.num_vertexes) * 4, 0); 
        out.weights_0.resize((mesh.first_vertex + mesh.num_vertexes) * 4, 0.0f);

        unsigned char* v_ptr = (unsigned char*)skm_buf + skm_meshes[m].ofs_verts;
        for (uint32_t v = 0; v < mesh.num_vertexes; ++v) {
            unsigned int num_influences = *(unsigned int*)v_ptr;
            v_ptr += sizeof(unsigned int);
            dskmbonevert_t* influences = (dskmbonevert_t*)v_ptr;
            v_ptr += num_influences * sizeof(dskmbonevert_t);

            float zup_pos[3] = {0, 0, 0};
            float zup_norm[3] = {0, 0, 0};

            struct Influence { int bone; float weight; };
            std::vector<Influence> sorted_influences;
            for (uint32_t i = 0; i < num_influences; ++i) {
                if (influences[i].bonenum < num_bones) {
                    transform_vec(zup_world_matrices[influences[i].bonenum], influences[i].origin, zup_pos, influences[i].influence);
                    rotate_vec(zup_world_matrices[influences[i].bonenum], influences[i].normal, zup_norm);
                    sorted_influences.push_back({(int)influences[i].bonenum, influences[i].influence});
                }
            }

            std::sort(sorted_influences.begin(), sorted_influences.end(), [](const Influence& a, const Influence& b) {
                return a.weight > b.weight;
            });

            float total_weight = 0;
            int n_infl = std::min((int)sorted_influences.size(), 4);
            for (int i = 0; i < n_infl; ++i) total_weight += sorted_influences[i].weight;

            for (int i = 0; i < n_infl; ++i) {
                out.joints_0[(mesh.first_vertex + v) * 4 + i] = (uint8_t)sorted_influences[i].bone;
                out.weights_0[(mesh.first_vertex + v) * 4 + i] = (total_weight > 0) ? (sorted_influences[i].weight / total_weight) : 0;
            }

            out.positions[(mesh.first_vertex + v) * 3 + 0] = zup_pos[0];     
            out.positions[(mesh.first_vertex + v) * 3 + 1] = zup_pos[1];     
            out.positions[(mesh.first_vertex + v) * 3 + 2] = zup_pos[2];     

            float norm_len = std::sqrt(zup_norm[0]*zup_norm[0] + zup_norm[1]*zup_norm[1] + zup_norm[2]*zup_norm[2]);
            if (norm_len > 1e-6f) {
                float inv_len = 1.0f / norm_len;
                zup_norm[0] *= inv_len; zup_norm[1] *= inv_len; zup_norm[2] *= inv_len;
            }
            out.normals[(mesh.first_vertex + v) * 3 + 0] = zup_norm[0];      
            out.normals[(mesh.first_vertex + v) * 3 + 1] = zup_norm[1];      
            out.normals[(mesh.first_vertex + v) * 3 + 2] = zup_norm[2];      
        }

        dskmcoord_t* st = (dskmcoord_t*)(skm_buf + skm_meshes[m].ofs_texcoords);
        for (uint32_t v = 0; v < mesh.num_vertexes; ++v) {
            out.texcoords[(mesh.first_vertex + v) * 2 + 0] = st[v].st[0];    
            out.texcoords[(mesh.first_vertex + v) * 2 + 1] = st[v].st[1];    
        }

        unsigned int* idx = (unsigned int*)(skm_buf + skm_meshes[m].ofs_indices);
        for (uint32_t t = 0; t < mesh.num_triangles; ++t) {
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 0]);       
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 1]);       
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 2]);       
        }
    }

    // Frames
    if (num_frames > 0) {
        struct TempAnim { std::string name; int first, count; float fps; };  
        std::vector<TempAnim> anims;

        if (external_anims && num_external_anims > 0) {
            uint32_t n = (num_external_anims > 1024) ? 1024 : num_external_anims;
            for (uint32_t i = 0; i < n; ++i) {
                int first = external_anims[i].first_frame;
                int count = external_anims[i].num_frames;
                if (first >= (int)num_frames) continue;
                if (first < 0) first = 0;
                if (first + count > (int)num_frames) {
                    count = (int)num_frames - first;
                }
                if (count <= 0) continue;
                anims.push_back({ external_anims[i].name ? external_anims[i].name : "unnamed", first, count, external_anims[i].fps });
            }
        } else {
            anims.push_back({"all", 0, (int)num_frames, BASE_FPS});
        }

        for (const auto& a : anims) {
            AnimationDef ad;
            ad.name = a.name;
            ad.duration = (double)a.count / a.fps;
            ad.bones.resize(num_bones);
            for (int f = a.first; f < a.first + a.count; ++f) {
                if (f < 0 || f >= (int)num_frames) continue;
                double time = (double)(f - a.first) / a.fps;

                for (uint32_t p = 0; p < num_bones; ++p) {
                    BoneAnim& ba = ad.bones[p];
                    float t[3]; std::memcpy(t, frame_poses[f][p].origin, 12);
                    float r[4]; std::memcpy(r, frame_poses[f][p].quat, 16);  

                    if (out.joints[p].parent == -1) {
                        quat_normalize(r);
                    }

                    ba.translation.times.push_back(time);
                    ba.translation.values.push_back(t[0]);
                    ba.translation.values.push_back(t[1]);
                    ba.translation.values.push_back(t[2]);
                    ba.rotation.times.push_back(time);
                    ba.rotation.values.push_back(r[0]);
                    ba.rotation.values.push_back(r[1]);
                    ba.rotation.values.push_back(r[2]);
                    ba.rotation.values.push_back(r[3]);
                    ba.scale.times.push_back(time);
                    ba.scale.values.push_back(1.0f);
                    ba.scale.values.push_back(1.0f);
                    ba.scale.values.push_back(1.0f);
                }
            }
            out.animations.push_back(ad);
        }
    }

    out.orientation = GS_Z_UP_RIGHTHANDED_X_FWD;
    out.winding = GS_WINDING_CW;
    return true;
}
