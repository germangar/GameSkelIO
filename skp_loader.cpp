#include "skp_loader.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>

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

bool load_skm(const char* path, Model& out) {
    std::string base_path = path;
    size_t dot = base_path.find_last_of('.');
    if (dot != std::string::npos && (base_path.substr(dot) == ".skm" || base_path.substr(dot) == ".skp")) {
        base_path = base_path.substr(0, dot);
    }

    std::string skm_path = base_path + ".skm";
    std::string skp_path = base_path + ".skp";

    std::ifstream f_skp(skp_path, std::ios::binary);
    if (!f_skp) {
        std::cerr << "Failed to open SKP file: " << skp_path << std::endl;
        return false;
    }
    f_skp.seekg(0, std::ios::end);
    size_t skp_size = f_skp.tellg();
    f_skp.seekg(0, std::ios::beg);
    std::vector<char> skp_buf(skp_size);
    f_skp.read(skp_buf.data(), skp_size);
    f_skp.close();

    dskpheader_t* skp_hdr = (dskpheader_t*)skp_buf.data();
    if (std::memcmp(skp_hdr->id, "SKM1", 4) != 0) {
        std::cerr << "Invalid SKP header magic" << std::endl;
        return false;
    }

    uint32_t num_bones = skp_hdr->num_bones;
    uint32_t num_frames = skp_hdr->num_frames;

    out.joints.resize(num_bones);
    dskpbone_t* skp_bones = (dskpbone_t*)(skp_buf.data() + skp_hdr->ofs_bones);
    for (uint32_t i = 0; i < num_bones; ++i) {
        out.joints[i].name = skp_bones[i].name;
        out.joints[i].parent = skp_bones[i].parent;
    }

    std::vector<std::vector<dskpbonepose_t>> frame_poses(num_frames, std::vector<dskpbonepose_t>(num_bones));
    dskpframe_t* skp_frames = (dskpframe_t*)(skp_buf.data() + skp_hdr->ofs_frames);
    for (uint32_t f = 0; f < num_frames; ++f) {
        dskpbonepose_t* poses = (dskpbonepose_t*)(skp_buf.data() + skp_frames[f].ofs_bonepositions);
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
            float t_yup[3] = {t[0], t[2], -t[1]};
            std::memcpy(t, t_yup, 12);
            float q_rot[4] = {-0.7071068f, 0.0f, 0.0f, 0.7071068f};      
            float r_new[4];
            quat_mul(q_rot, r, r_new);
            quat_normalize(r_new);
            std::memcpy(r, r_new, 16);
        }
        std::memcpy(out.joints[i].translate, t, 12);
        std::memcpy(out.joints[i].rotate, r, 16);
        std::memcpy(out.joints[i].scale, s, 12);
    }

    std::ifstream f_skm(skm_path, std::ios::binary);
    if (!f_skm) {
        std::cerr << "Failed to open SKM file: " << skm_path << std::endl;
        return false;
    }
    f_skm.seekg(0, std::ios::end);
    size_t skm_size = f_skm.tellg();
    f_skm.seekg(0, std::ios::beg);
    std::vector<char> skm_buf(skm_size);
    f_skm.read(skm_buf.data(), skm_size);
    f_skm.close();

    dskmheader_t* skm_hdr = (dskmheader_t*)skm_buf.data();
    if (std::memcmp(skm_hdr->id, "SKM1", 4) != 0) {
        std::cerr << "Invalid SKM header magic" << std::endl;
        return false;
    }

    dskmmesh_t* skm_meshes = (dskmmesh_t*)(skm_buf.data() + skm_hdr->ofs_meshes);
    out.meshes.resize(skm_hdr->num_meshes);

    for (uint32_t m = 0; m < skm_hdr->num_meshes; ++m) {
        Mesh& mesh = out.meshes[m];
        mesh.name = skm_meshes[m].meshname;
        mesh.material_name = skm_meshes[m].shadername;
        mesh.first_vertex = out.positions.size() / 3;
        mesh.num_vertexes = skm_meshes[m].num_verts;
        mesh.first_triangle = out.indices.size() / 3;
        mesh.num_triangles = skm_meshes[m].num_tris;

        out.positions.resize((mesh.first_vertex + mesh.num_vertexes) * 3);
        out.normals.resize((mesh.first_vertex + mesh.num_vertexes) * 3); 
        out.texcoords.resize((mesh.first_vertex + mesh.num_vertexes) * 2);
        out.joints_0.resize((mesh.first_vertex + mesh.num_vertexes) * 4, 0);
        out.weights_0.resize((mesh.first_vertex + mesh.num_vertexes) * 4, 0.0f);

        unsigned char* v_ptr = (unsigned char*)skm_buf.data() + skm_meshes[m].ofs_verts;
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
            out.positions[(mesh.first_vertex + v) * 3 + 1] = zup_pos[2]; 
            out.positions[(mesh.first_vertex + v) * 3 + 2] = -zup_pos[1];

            float norm_len = std::sqrt(zup_norm[0]*zup_norm[0] + zup_norm[1]*zup_norm[1] + zup_norm[2]*zup_norm[2]);
            if (norm_len > 1e-6f) {
                float inv_len = 1.0f / norm_len;
                zup_norm[0] *= inv_len; zup_norm[1] *= inv_len; zup_norm[2] *= inv_len;
            }
            out.normals[(mesh.first_vertex + v) * 3 + 0] = zup_norm[0];  
            out.normals[(mesh.first_vertex + v) * 3 + 1] = zup_norm[2];  
            out.normals[(mesh.first_vertex + v) * 3 + 2] = -zup_norm[1]; 
        }

        dskmcoord_t* st = (dskmcoord_t*)(skm_buf.data() + skm_meshes[m].ofs_texcoords);
        for (uint32_t v = 0; v < mesh.num_vertexes; ++v) {
            out.texcoords[(mesh.first_vertex + v) * 2 + 0] = st[v].st[0];
            out.texcoords[(mesh.first_vertex + v) * 2 + 1] = st[v].st[1];
        }

        unsigned int* idx = (unsigned int*)(skm_buf.data() + skm_meshes[m].ofs_indices);
        for (uint32_t t = 0; t < mesh.num_triangles; ++t) {
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 0]);   
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 2]);
            out.indices.push_back(mesh.first_vertex + idx[t * 3 + 1]);
        }
    }

    std::string cfg_path = find_animation_cfg(skm_path);
    std::vector<AnimConfigEntry> entries;
    if (!cfg_path.empty()) {
        std::cout << "Found animation config: " << cfg_path << std::endl;
        entries = parse_animation_cfg(cfg_path);
    } else {
        entries.push_back({"all", 0, (int)num_frames - 1, 0, BASE_FPS});
    }

    for (const auto& entry : entries) {
        AnimationDef ad;
        ad.name = entry.name;
        ad.track.bones.resize(num_bones);
        for (int f = entry.first_frame; f <= entry.last_frame; ++f) {      
            if (f < 0 || f >= (int)num_frames) continue;
            double time = (double)(f - entry.first_frame) / entry.fps;     

            for (uint32_t p = 0; p < num_bones; ++p) {
                BoneAnim& ba = ad.track.bones[p];
                float t[3]; std::memcpy(t, frame_poses[f][p].origin, 12);
                float r[4]; std::memcpy(r, frame_poses[f][p].quat, 16);  

                if (out.joints[p].parent == -1) {
                    float tx = t[0], ty = t[1], tz = t[2];
                    t[0] = tx; t[1] = tz; t[2] = -ty;
                    float q_rot[4] = {-0.7071068f, 0.0f, 0.0f, 0.7071068f};
                    float r_new[4];
                    quat_mul(q_rot, r, r_new);
                    quat_normalize(r_new);
                    std::memcpy(r, r_new, 16);
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

    return true;
}
