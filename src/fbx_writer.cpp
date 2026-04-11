#include "fbx_writer.h"
#include "fbx.hpp"
#include <map>
#include <ctime>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>

static std::string sanitize(const std::string& s) {
    std::string res = s;
    std::replace(res.begin(), res.end(), ' ', '_');
    return res;
}

// FBX constants
const int64_t KTIME_ONE_SECOND = 46186158000LL;

// Helper to generate a unique 64-bit ID for FBX nodes
static int64_t generate_id() {
    static int64_t last_id = 2000000;
    return ++last_id;
}

bool write_fbx(const char* path, const Model& in, bool write_mesh, bool write_anim, int anim_index) {
    Fbx::Record file;
    
    // 1. Header Extension
    Fbx::Record* hdr_ext = new Fbx::Record("FBXHeaderExtension", &file);
    (*hdr_ext->insert(new Fbx::Record("FBXHeaderVersion")))->properties().insert(new Fbx::Property((int32_t)1003));
    (*hdr_ext->insert(new Fbx::Record("FBXVersion")))->properties().insert(new Fbx::Property((int32_t)7400));
    
    time_t now = time(0);
    tm* ltm = localtime(&now);
    Fbx::Record* creation_time = new Fbx::Record("CreationTimeStamp", hdr_ext);
    (*creation_time->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)1000));
    (*creation_time->insert(new Fbx::Record("Year")))->properties().insert(new Fbx::Property((int32_t)(1900 + ltm->tm_year)));
    (*creation_time->insert(new Fbx::Record("Month")))->properties().insert(new Fbx::Property((int32_t)(1 + ltm->tm_mon)));
    (*creation_time->insert(new Fbx::Record("Day")))->properties().insert(new Fbx::Property((int32_t)ltm->tm_mday));
    (*creation_time->insert(new Fbx::Record("Hour")))->properties().insert(new Fbx::Property((int32_t)ltm->tm_hour));
    (*creation_time->insert(new Fbx::Record("Minute")))->properties().insert(new Fbx::Property((int32_t)ltm->tm_min));
    (*creation_time->insert(new Fbx::Record("Second")))->properties().insert(new Fbx::Property((int32_t)ltm->tm_sec));
    (*creation_time->insert(new Fbx::Record("Millisecond")))->properties().insert(new Fbx::Property((int32_t)0));
    
    (*hdr_ext->insert(new Fbx::Record("Creator")))->properties().insert(new Fbx::Property("GameSkelIO - FBX Writer"));
    
    Fbx::Record* metadata = new Fbx::Record("Metadata", hdr_ext);
    (*metadata->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)100));
    (*metadata->insert(new Fbx::Record("Title")))->properties().insert(new Fbx::Property(""));
    (*metadata->insert(new Fbx::Record("Subject")))->properties().insert(new Fbx::Property(""));
    (*metadata->insert(new Fbx::Record("Author")))->properties().insert(new Fbx::Property(""));
    (*metadata->insert(new Fbx::Record("Keywords")))->properties().insert(new Fbx::Property(""));
    (*metadata->insert(new Fbx::Record("Revision")))->properties().insert(new Fbx::Property(""));
    (*metadata->insert(new Fbx::Record("Comment")))->properties().insert(new Fbx::Property(""));

    // 2. GlobalSettings
    Fbx::Record* gs = new Fbx::Record("GlobalSettings", &file);
    (*gs->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)1000));
    Fbx::Record* props70 = new Fbx::Record("Properties70", gs);
    
    auto add_prop70 = [&](Fbx::Record* parent, const char* name, const char* type, const char* label, const char* flag, auto value) {
        Fbx::Record* p = new Fbx::Record("P", parent);
        p->properties().insert(new Fbx::Property(name));
        p->properties().insert(new Fbx::Property(type));
        p->properties().insert(new Fbx::Property(label));
        p->properties().insert(new Fbx::Property(flag));
        p->properties().insert(new Fbx::Property(value));
    };

    auto add_prop70_3d = [&](Fbx::Record* parent, const char* name, const char* type, const char* label, const char* flag, double x, double y, double z) {
        Fbx::Record* p = new Fbx::Record("P", parent);
        p->properties().insert(new Fbx::Property(name));
        p->properties().insert(new Fbx::Property(type));
        p->properties().insert(new Fbx::Property(label));
        p->properties().insert(new Fbx::Property(flag));
        p->properties().insert(new Fbx::Property(x));
        p->properties().insert(new Fbx::Property(y));
        p->properties().insert(new Fbx::Property(z));
    };

    add_prop70(props70, "UpAxis", "int", "Integer", "", (int32_t)1); // Y-up
    add_prop70(props70, "UpAxisSign", "int", "Integer", "", (int32_t)1);
    add_prop70(props70, "FrontAxis", "int", "Integer", "", (int32_t)2); // Z
    add_prop70(props70, "FrontAxisSign", "int", "Integer", "", (int32_t)1);
    add_prop70(props70, "CoordAxis", "int", "Integer", "", (int32_t)0); // Right
    add_prop70(props70, "CoordAxisSign", "int", "Integer", "", (int32_t)1);
    add_prop70(props70, "OriginalUpAxis", "int", "Integer", "", (int32_t)1);
    add_prop70(props70, "OriginalUpAxisSign", "int", "Integer", "", (int32_t)1);
    add_prop70(props70, "UnitScaleFactor", "double", "Number", "", (double)100.0);
    add_prop70(props70, "OriginalUnitScaleFactor", "double", "Number", "", (double)100.0);

    // 3. Documents
    Fbx::Record* docs = new Fbx::Record("Documents", &file);
    (*docs->insert(new Fbx::Record("Count")))->properties().insert(new Fbx::Property((int32_t)1));
    Fbx::Record* doc = new Fbx::Record("Document", docs);
    int64_t doc_id = generate_id();
    doc->properties().insert(new Fbx::Property(doc_id));
    doc->properties().insert(new Fbx::Property(""));
    doc->properties().insert(new Fbx::Property("Scene"));

    // 4. References (Empty)
    new Fbx::Record("References", &file);

    // 5. Definitions
    Fbx::Record* defs = new Fbx::Record("Definitions", &file);
    (*defs->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)100));
    
    int32_t num_models = (int32_t)in.joints.size() + (write_mesh ? (int32_t)in.meshes.size() : 0);
    int32_t num_node_attributes = (int32_t)in.joints.size();
    int32_t num_geoms = write_mesh ? (int32_t)in.meshes.size() : 0;
    
    std::vector<std::string> unique_textures;
    if (write_mesh) {
        auto add_unique = [&](std::vector<std::string>& list, const std::string& val) {
            if (val.empty()) return;
            if (std::find(list.begin(), list.end(), val) == list.end()) list.push_back(val);
        };
        for (const auto& m : in.materials) {
            add_unique(unique_textures, m.color_map);
            add_unique(unique_textures, m.normal_map);
            add_unique(unique_textures, m.metallic_map);
            add_unique(unique_textures, m.roughness_map);
            add_unique(unique_textures, m.specular_map);
            add_unique(unique_textures, m.shininess_map);
            add_unique(unique_textures, m.emissive_map);
            add_unique(unique_textures, m.occlusion_map);
            add_unique(unique_textures, m.opacity_map);
        }
    }
    int32_t num_mats = write_mesh ? (int32_t)in.materials.size() : 0;
    int32_t num_textures = (int32_t)unique_textures.size();
    int32_t num_videos = num_textures;
    
    int32_t num_skins = 0;
    int32_t num_valid_clusters_total = 0;
    if (write_mesh) {
        bool global_has_skin = (in.joints_0.size() >= (in.positions.size() / 3 * 4)) && (in.weights_0.size() >= (in.positions.size() / 3 * 4));
        if (global_has_skin) {
            for (size_t mi = 0; mi < in.meshes.size(); ++mi) {
                num_skins++;
                for (size_t i = 0; i < in.joints.size(); ++i) {
                    bool has_weight = false;
                    for (uint32_t v = 0; v < in.meshes[mi].num_vertexes; ++v) {
                        uint32_t gv = in.meshes[mi].first_vertex + v;
                        for (int k = 0; k < 4; ++k) {
                            if (in.joints_0[gv*4+k] == (uint8_t)i && in.weights_0[gv*4+k] > 0.001f) {
                                has_weight = true;
                                break;
                            }
                        }
                        if (has_weight) break;
                    }
                    if (has_weight) num_valid_clusters_total++;
                }
            }
        }
    }
    int32_t num_deformers = num_skins + num_valid_clusters_total;

    int32_t actual_anims = (write_anim) ? (anim_index >= 0 ? 1 : (int32_t)in.animations.size()) : 0;
    int32_t num_poses = 1;
    int32_t num_joints = (int32_t)in.joints.size();
    int32_t num_anim_stacks = actual_anims;
    int32_t num_anim_layers = actual_anims;
    int32_t num_anim_curve_nodes = actual_anims * num_joints * 3;
    int32_t num_anim_curves = actual_anims * num_joints * 9;
    
    int32_t total_objs = num_models + num_node_attributes + num_geoms + num_mats + num_deformers + num_poses + 
                         num_anim_stacks + num_anim_layers + num_anim_curve_nodes + num_anim_curves +
                         num_textures + num_videos;
                         
    (*defs->insert(new Fbx::Record("Count")))->properties().insert(new Fbx::Property(total_objs));
    
    auto add_def = [&](const char* type, int32_t count) {
        if (count > 0) {
            Fbx::Record* ot = new Fbx::Record("ObjectType", defs);
            ot->properties().insert(new Fbx::Property(type));
            (*ot->insert(new Fbx::Record("Count")))->properties().insert(new Fbx::Property(count));
        }
    };
    
    add_def("Model", num_models);
    add_def("NodeAttribute", num_node_attributes);
    add_def("Geometry", num_geoms);
    add_def("Material", num_mats);
    add_def("Texture", num_textures);
    add_def("Video", num_videos);
    add_def("Deformer", num_deformers);
    add_def("Pose", num_poses);
    add_def("AnimationStack", num_anim_stacks);
    add_def("AnimationLayer", num_anim_layers);
    add_def("AnimationCurveNode", num_anim_curve_nodes);
    add_def("AnimationCurve", num_anim_curves);

    // 6. Objects
    Fbx::Record* objs = new Fbx::Record("Objects", &file);
    
    std::map<int, int64_t> joint_to_id;
    std::map<int, int64_t> joint_to_attr_id;
    
    std::vector<int64_t> mesh_model_ids(in.meshes.size());
    std::vector<int64_t> mesh_geom_ids(in.meshes.size());
    std::vector<int64_t> skin_ids(in.meshes.size());
    for(size_t i=0; i<in.meshes.size(); ++i) {
        mesh_model_ids[i] = generate_id();
        mesh_geom_ids[i] = generate_id();
        skin_ids[i] = generate_id();
    }

    struct FBXMaterialIDs { 
        int64_t id; 
        int64_t color_id; 
        int64_t normal_id; 
        int64_t metallic_id;
        int64_t rough_id; 
        int64_t spec_id;
        int64_t shininess_id;
        int64_t emissive_id;
        int64_t occ_id;
        int64_t opacity_id;
    };
    std::vector<FBXMaterialIDs> material_ids(in.materials.size());
    std::map<std::string, int64_t> texture_to_id;
    std::map<std::string, int64_t> texture_to_video_id;

    if (write_mesh) {
        // Create Video and Texture objects for all unique textures
        for (const auto& path : unique_textures) {
            int64_t vid_id = generate_id();
            int64_t tex_id = generate_id();
            texture_to_id[path] = tex_id;
            texture_to_video_id[path] = vid_id;
            
            Fbx::Record* vid = new Fbx::Record("Video", objs);
            vid->properties().insert(new Fbx::Property(vid_id));
            vid->properties().insert(new Fbx::Property(sanitize(path) + std::string("\x00\x01", 2) + "Video"));
            vid->properties().insert(new Fbx::Property("Clip"));
            (*vid->insert(new Fbx::Record("Type")))->properties().insert(new Fbx::Property("Clip"));
            (*vid->insert(new Fbx::Record("FileName")))->properties().insert(new Fbx::Property(path));
            (*vid->insert(new Fbx::Record("RelativeFilename")))->properties().insert(new Fbx::Property(path));

            Fbx::Record* tex = new Fbx::Record("Texture", objs);
            tex->properties().insert(new Fbx::Property(tex_id));
            tex->properties().insert(new Fbx::Property(sanitize(path) + std::string("\x00\x01", 2) + "Texture"));
            tex->properties().insert(new Fbx::Property(""));
            (*tex->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)202));
            (*tex->insert(new Fbx::Record("FileName")))->properties().insert(new Fbx::Property(path));
            (*tex->insert(new Fbx::Record("RelativeFilename")))->properties().insert(new Fbx::Property(path));
            Fbx::Record* p70_tex = new Fbx::Record("Properties70", tex);
            add_prop70(p70_tex, "CurrentTextureBlendMode", "enum", "", "", (int32_t)0);
            add_prop70(p70_tex, "UVSet", "KString", "", "", "map1");
        }

        auto get_tex_id = [&](const std::string& path) {
            if (path.empty()) return (int64_t)0;
            return texture_to_id[path];
        };

        for (size_t mi = 0; mi < in.materials.size(); ++mi) {
            const Material& m = in.materials[mi];
            FBXMaterialIDs mids = {};
            mids.id = generate_id();
            mids.color_id = get_tex_id(m.color_map);
            mids.normal_id = get_tex_id(m.normal_map);
            mids.metallic_id = get_tex_id(m.metallic_map);
            mids.rough_id = get_tex_id(m.roughness_map);
            mids.spec_id = get_tex_id(m.specular_map);
            mids.shininess_id = get_tex_id(m.shininess_map);
            mids.emissive_id = get_tex_id(m.emissive_map);
            mids.occ_id = get_tex_id(m.occlusion_map);
            mids.opacity_id = get_tex_id(m.opacity_map);
            material_ids[mi] = mids;

            Fbx::Record* mat = new Fbx::Record("Material", objs);
            mat->properties().insert(new Fbx::Property(mids.id));
            mat->properties().insert(new Fbx::Property(sanitize(m.name) + std::string("\x00\x01", 2) + "Material"));
            mat->properties().insert(new Fbx::Property(""));
            (*mat->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)102));
            
            if (m.material_type == 0) {
                (*mat->insert(new Fbx::Record("ShadingModel")))->properties().insert(new Fbx::Property("pbs"));
            } else {
                (*mat->insert(new Fbx::Record("ShadingModel")))->properties().insert(new Fbx::Property("phong"));
            }
            
            (*mat->insert(new Fbx::Record("MultiLayer")))->properties().insert(new Fbx::Property((int32_t)0));

            Fbx::Record* p70_mat = new Fbx::Record("Properties70", mat);
            if (m.material_type == 0) {
                add_prop70_3d(p70_mat, "BaseColor", "Color", "", "A", (double)m.base_color[0], (double)m.base_color[1], (double)m.base_color[2]);
                add_prop70(p70_mat, "Metallic", "double", "Number", "A", (double)m.metallic_factor);
                add_prop70(p70_mat, "Roughness", "double", "Number", "A", (double)m.roughness_factor);
                add_prop70_3d(p70_mat, "EmissiveColor", "Color", "", "A", (double)m.emissive_color[0], (double)m.emissive_color[1], (double)m.emissive_color[2]);
                add_prop70(p70_mat, "EmissiveFactor", "double", "Number", "A", (double)m.emissive_factor);
            } else {
                add_prop70_3d(p70_mat, "DiffuseColor", "Color", "", "A", (double)m.base_color[0], (double)m.base_color[1], (double)m.base_color[2]);
                add_prop70_3d(p70_mat, "SpecularColor", "Color", "", "A", (double)m.specular_color[0], (double)m.specular_color[1], (double)m.specular_color[2]);
                add_prop70_3d(p70_mat, "EmissiveColor", "Color", "", "A", (double)m.emissive_color[0], (double)m.emissive_color[1], (double)m.emissive_color[2]);
            }
        }
    }

    std::vector<std::vector<int64_t>> mesh_valid_cluster_ids(in.meshes.size());
    std::vector<std::vector<int64_t>> mesh_valid_joint_ids(in.meshes.size());

    // A. Joints (LimbNodes)
    for (size_t i = 0; i < in.joints.size(); ++i) {
        int64_t j_id = generate_id();
        joint_to_id[(int)i] = j_id;
        
        Fbx::Record* mod = new Fbx::Record("Model", objs);
        mod->properties().insert(new Fbx::Property(j_id));
        mod->properties().insert(new Fbx::Property(sanitize(in.joints[i].name) + std::string("\x00\x01", 2) + "Model"));
        mod->properties().insert(new Fbx::Property("LimbNode"));
        
        (*mod->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)232));
        Fbx::Record* p70 = new Fbx::Record("Properties70", mod);
        
        add_prop70_3d(p70, "Lcl Translation", "Lcl Translation", "", "A", (double)in.joints[i].translate[0], (double)in.joints[i].translate[1], (double)in.joints[i].translate[2]);
        
        float t[3], q[4], s[3];
        memcpy(t, in.joints[i].translate, 12);
        memcpy(q, in.joints[i].rotate, 16);
        memcpy(s, in.joints[i].scale, 12);
        stabilize_trs(t, q, s);

        float euler[3]; 
        quat_to_euler(q, euler);
        add_prop70_3d(p70, "Lcl Rotation", "Lcl Rotation", "", "A", (double)euler[0], (double)euler[1], (double)euler[2]);
        add_prop70_3d(p70, "Lcl Scaling", "Lcl Scaling", "", "A", (double)s[0], (double)s[1], (double)s[2]);
        add_prop70(p70, "RotationOrder", "enum", "", "", (int32_t)0);
        
        (*mod->insert(new Fbx::Record("Shading")))->properties().insert(new Fbx::Property(true));
        (*mod->insert(new Fbx::Record("Culling")))->properties().insert(new Fbx::Property("CullingOff"));

        int64_t attr_id = generate_id();
        joint_to_attr_id[(int)i] = attr_id;
        Fbx::Record* attr = new Fbx::Record("NodeAttribute", objs);
        attr->properties().insert(new Fbx::Property(attr_id));
        attr->properties().insert(new Fbx::Property(sanitize(in.joints[i].name) + std::string("\x00\x01", 2) + "NodeAttribute"));
        attr->properties().insert(new Fbx::Property("LimbNode"));
        (*attr->insert(new Fbx::Record("TypeFlags")))->properties().insert(new Fbx::Property("Skeleton"));
    }

    if (write_mesh) {
        for (size_t mi = 0; mi < in.meshes.size(); ++mi) {
            // B. Geometry
            Fbx::Record* geom = new Fbx::Record("Geometry", objs);
            geom->properties().insert(new Fbx::Property(mesh_geom_ids[mi]));
            geom->properties().insert(new Fbx::Property(sanitize(in.meshes[mi].name) + std::string("\x00\x01", 2) + "Geometry"));
            geom->properties().insert(new Fbx::Property("Mesh"));
            
            std::vector<double> verts;
            for (uint32_t v = 0; v < in.meshes[mi].num_vertexes; ++v) {
                uint32_t gv = in.meshes[mi].first_vertex + v;
                if (gv * 3 + 2 < in.positions.size()) {
                    verts.push_back((double)in.positions[gv*3+0]);
                    verts.push_back((double)in.positions[gv*3+1]);
                    verts.push_back((double)in.positions[gv*3+2]);
                } else {
                    verts.push_back(0.0); verts.push_back(0.0); verts.push_back(0.0);
                }
            }
            (*geom->insert(new Fbx::Record("Vertices")))->properties().insert(new Fbx::Property(verts.data(), (uint32_t)verts.size()));
            
            std::vector<int32_t> poly_indices;
            for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                uint32_t gt = in.meshes[mi].first_triangle + i;
                if (gt * 3 + 2 >= in.indices.size()) break;
                poly_indices.push_back((int32_t)(in.indices[gt*3+0] - in.meshes[mi].first_vertex));
                poly_indices.push_back((int32_t)(in.indices[gt*3+1] - in.meshes[mi].first_vertex));
                poly_indices.push_back((int32_t)(in.indices[gt*3+2] - in.meshes[mi].first_vertex) ^ -1);
            }
            (*geom->insert(new Fbx::Record("PolygonVertexIndex")))->properties().insert(new Fbx::Property(poly_indices.data(), (uint32_t)poly_indices.size()));

            // Normals
            Fbx::Record* layer_norm = new Fbx::Record("LayerElementNormal", geom);
            layer_norm->properties().insert(new Fbx::Property((int32_t)0));
            (*layer_norm->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
            (*layer_norm->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
            (*layer_norm->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
            std::vector<double> norms;
            for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                uint32_t gt = in.meshes[mi].first_triangle + i;
                if (gt * 3 + 2 >= in.indices.size()) break;
                for (int k = 0; k < 3; ++k) {
                    uint32_t idx = in.indices[gt*3+k];
                    if (idx * 3 + 2 < in.normals.size()) {
                        norms.push_back((double)in.normals[idx*3+0]);
                        norms.push_back((double)in.normals[idx*3+1]);
                        norms.push_back((double)in.normals[idx*3+2]);
                    } else {
                        norms.push_back(0.0); norms.push_back(0.0); norms.push_back(0.0);
                    }
                }
            }
            (*layer_norm->insert(new Fbx::Record("Normals")))->properties().insert(new Fbx::Property(norms.data(), (uint32_t)norms.size()));

            // UVs
            Fbx::Record* layer_uv = new Fbx::Record("LayerElementUV", geom);
            layer_uv->properties().insert(new Fbx::Property((int32_t)0));
            (*layer_uv->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
            (*layer_uv->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
            (*layer_uv->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
            std::vector<double> uv_data;
            for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                uint32_t gt = in.meshes[mi].first_triangle + i;
                if (gt * 3 + 2 >= in.indices.size()) break;
                for (int k = 0; k < 3; ++k) {
                    uint32_t idx = in.indices[gt*3+k];
                    if (idx * 2 + 1 < in.texcoords.size()) {
                        uv_data.push_back((double)in.texcoords[idx*2]);
                        uv_data.push_back((double)(1.0f - in.texcoords[idx*2+1]));
                    } else {
                        uv_data.push_back(0.0); uv_data.push_back(0.0);
                    }
                }
            }
            (*layer_uv->insert(new Fbx::Record("UV")))->properties().insert(new Fbx::Property(uv_data.data(), (uint32_t)uv_data.size()));

            // Tangents & Binormals
            if (!in.tangents.empty() && in.tangents.size() >= (in.positions.size() / 3 * 4)) {
                Fbx::Record* layer_tan = new Fbx::Record("LayerElementTangent", geom);
                layer_tan->properties().insert(new Fbx::Property((int32_t)0));
                (*layer_tan->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
                (*layer_tan->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
                (*layer_tan->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
                std::vector<double> tan_data;
                std::vector<double> bitan_data;
                for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                    uint32_t gt = in.meshes[mi].first_triangle + i;
                    if (gt * 3 + 2 >= in.indices.size()) break;
                    for (int k = 0; k < 3; ++k) {
                        uint32_t idx = in.indices[gt*3+k];
                        if (idx * 4 + 3 < in.tangents.size()) {
                            tan_data.push_back((double)in.tangents[idx*4 + 0]);
                            tan_data.push_back((double)in.tangents[idx*4 + 1]);
                            tan_data.push_back((double)in.tangents[idx*4 + 2]);
                            
                            float n[3] = {0,0,0};
                            if (idx * 3 + 2 < in.normals.size()) {
                                n[0] = in.normals[idx*3+0]; n[1] = in.normals[idx*3+1]; n[2] = in.normals[idx*3+2];
                            }
                            float t[3] = {in.tangents[idx*4+0], in.tangents[idx*4+1], in.tangents[idx*4+2]};
                            float w = in.tangents[idx*4+3];
                            float b[3] = { (n[1]*t[2] - n[2]*t[1]) * w, (n[2]*t[0] - n[0]*t[2]) * w, (n[0]*t[1] - n[1]*t[0]) * w };
                            bitan_data.push_back((double)b[0]);
                            bitan_data.push_back((double)b[1]);
                            bitan_data.push_back((double)b[2]);
                        } else {
                            tan_data.push_back(0.0); tan_data.push_back(0.0); tan_data.push_back(0.0);
                            bitan_data.push_back(0.0); bitan_data.push_back(0.0); bitan_data.push_back(0.0);
                        }
                    }
                }
                (*layer_tan->insert(new Fbx::Record("Tangents")))->properties().insert(new Fbx::Property(tan_data.data(), (uint32_t)tan_data.size()));

                Fbx::Record* layer_bitan = new Fbx::Record("LayerElementBinormal", geom);
                layer_bitan->properties().insert(new Fbx::Property((int32_t)0));
                (*layer_bitan->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
                (*layer_bitan->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
                (*layer_bitan->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
                (*layer_bitan->insert(new Fbx::Record("Binormals")))->properties().insert(new Fbx::Property(bitan_data.data(), (uint32_t)bitan_data.size()));
            }

            // Vertex Colors
            if (!in.colors.empty() && in.colors.size() >= (in.positions.size() / 3 * 4)) {
                Fbx::Record* layer_col = new Fbx::Record("LayerElementColor", geom);
                layer_col->properties().insert(new Fbx::Property((int32_t)0));
                (*layer_col->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
                (*layer_col->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
                (*layer_col->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
                std::vector<double> col_data;
                for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                    uint32_t gt = in.meshes[mi].first_triangle + i;
                    if (gt * 3 + 2 >= in.indices.size()) break;
                    for (int k = 0; k < 3; ++k) {
                        uint32_t idx = in.indices[gt*3+k];
                        if (idx * 4 + 3 < in.colors.size()) {
                            col_data.push_back((double)in.colors[idx*4 + 0]);
                            col_data.push_back((double)in.colors[idx*4 + 1]);
                            col_data.push_back((double)in.colors[idx*4 + 2]);
                            col_data.push_back((double)in.colors[idx*4 + 3]);
                        } else {
                            col_data.push_back(0.0); col_data.push_back(0.0); col_data.push_back(0.0); col_data.push_back(0.0);
                        }
                    }
                }
                (*layer_col->insert(new Fbx::Record("Colors")))->properties().insert(new Fbx::Property(col_data.data(), (uint32_t)col_data.size()));
            }

            // Secondary UVs
            if (!in.texcoords_1.empty() && in.texcoords_1.size() >= (in.positions.size() / 3 * 2)) {
                Fbx::Record* layer_uv1 = new Fbx::Record("LayerElementUV", geom);
                layer_uv1->properties().insert(new Fbx::Property((int32_t)1));
                (*layer_uv1->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
                (*layer_uv1->insert(new Fbx::Record("Name")))->properties().insert(new Fbx::Property("uv1"));
                (*layer_uv1->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("ByPolygonVertex"));
                (*layer_uv1->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("Direct"));
                std::vector<double> uv1_data;
                for (size_t i = 0; i < in.meshes[mi].num_triangles; ++i) {
                    uint32_t gt = in.meshes[mi].first_triangle + i;
                    if (gt * 3 + 2 >= in.indices.size()) break;
                    for (int k = 0; k < 3; ++k) {
                        uint32_t idx = in.indices[gt*3+k];
                        if (idx * 2 + 1 < in.texcoords_1.size()) {
                            uv1_data.push_back((double)in.texcoords_1[idx*2 + 0]);
                            uv1_data.push_back((double)(1.0f - in.texcoords_1[idx*2 + 1]));
                        } else {
                            uv1_data.push_back(0.0); uv1_data.push_back(0.0);
                        }
                    }
                }
                (*layer_uv1->insert(new Fbx::Record("UV")))->properties().insert(new Fbx::Property(uv1_data.data(), (uint32_t)uv1_data.size()));
            }

            // Materials
            Fbx::Record* layer_mat = new Fbx::Record("LayerElementMaterial", geom);
            layer_mat->properties().insert(new Fbx::Property((int32_t)0));
            (*layer_mat->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
            (*layer_mat->insert(new Fbx::Record("MappingInformationType")))->properties().insert(new Fbx::Property("AllSame"));
            (*layer_mat->insert(new Fbx::Record("ReferenceInformationType")))->properties().insert(new Fbx::Property("IndexToDirect"));
            std::vector<int32_t> mat_indices = {0};
            (*layer_mat->insert(new Fbx::Record("Materials")))->properties().insert(new Fbx::Property(mat_indices.data(), 1));

            // Layer 0
            Fbx::Record* layer = new Fbx::Record("Layer", geom);
            layer->properties().insert(new Fbx::Property((int32_t)0));
            (*layer->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)100));
            auto add_le = [&](const char* type, int32_t idx) {
                Fbx::Record* le = new Fbx::Record("LayerElement", layer);
                (*le->insert(new Fbx::Record("Type")))->properties().insert(new Fbx::Property(type));
                (*le->insert(new Fbx::Record("TypedIndex")))->properties().insert(new Fbx::Property(idx));
            };
            add_le("LayerElementNormal", 0);
            add_le("LayerElementUV", 0);
            if (!in.tangents.empty()) { add_le("LayerElementTangent", 0); add_le("LayerElementBinormal", 0); }
            if (!in.colors.empty()) add_le("LayerElementColor", 0);
            if (!in.texcoords_1.empty()) add_le("LayerElementUV", 1);
            add_le("LayerElementMaterial", 0);

            // Mesh Model
            Fbx::Record* mod_mesh = new Fbx::Record("Model", objs);
            mod_mesh->properties().insert(new Fbx::Property(mesh_model_ids[mi]));
            mod_mesh->properties().insert(new Fbx::Property(sanitize(in.meshes[mi].name) + std::string("\x00\x01", 2) + "Model"));
            mod_mesh->properties().insert(new Fbx::Property("Mesh"));
            (*mod_mesh->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)232));
            Fbx::Record* p70_mesh = new Fbx::Record("Properties70", mod_mesh);
            add_prop70(p70_mesh, "RotationOrder", "enum", "", "", (int32_t)0);

            // Skin & Clusters
            bool global_has_skin = (in.joints_0.size() >= (in.positions.size() / 3 * 4)) && (in.weights_0.size() >= (in.positions.size() / 3 * 4));
            if (global_has_skin) {
                Fbx::Record* skin_rec = new Fbx::Record("Deformer", objs);
                skin_rec->properties().insert(new Fbx::Property(skin_ids[mi]));
                skin_rec->properties().insert(new Fbx::Property("Skin" + std::string("\x00\x01", 2) + "Deformer"));
                skin_rec->properties().insert(new Fbx::Property("Skin"));
                (*skin_rec->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)101));
                (*skin_rec->insert(new Fbx::Record("Link_DeformAcuracy")))->properties().insert(new Fbx::Property((double)50.0));
                
                for (size_t i = 0; i < in.joints.size(); ++i) {
                    std::vector<int32_t> idxs;
                    std::vector<double> wts;
                    for (uint32_t v = 0; v < in.meshes[mi].num_vertexes; ++v) {
                        uint32_t gv = in.meshes[mi].first_vertex + v;
                        if (gv * 4 + 3 >= in.joints_0.size() || gv * 4 + 3 >= in.weights_0.size()) continue;
                        for (int k = 0; k < 4; ++k) {
                            if (in.joints_0[gv*4+k] == (uint8_t)i && in.weights_0[gv*4+k] > 0.001f) {
                                idxs.push_back((int32_t)v);
                                wts.push_back((double)in.weights_0[gv*4+k]);
                                break;
                            }
                        }
                    }

                    if (idxs.empty()) continue;

                    int64_t c_id = generate_id();
                    mesh_valid_cluster_ids[mi].push_back(c_id);
                    mesh_valid_joint_ids[mi].push_back(joint_to_id[(int)i]);

                    Fbx::Record* cluster = new Fbx::Record("Deformer", objs);
                    cluster->properties().insert(new Fbx::Property(c_id));
                    cluster->properties().insert(new Fbx::Property("Cluster_" + in.joints[i].name + std::string("\x00\x01", 2) + "SubDeformer"));
                    cluster->properties().insert(new Fbx::Property("Cluster"));
                    (*cluster->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)100));
                    
                    Fbx::Record* userdata = new Fbx::Record("UserData", cluster);
                    userdata->properties().insert(new Fbx::Property(""));
                    userdata->properties().insert(new Fbx::Property(""));

                    (*cluster->insert(new Fbx::Record("Indexes")))->properties().insert(new Fbx::Property(idxs.data(), (uint32_t)idxs.size()));
                    (*cluster->insert(new Fbx::Record("Weights")))->properties().insert(new Fbx::Property(wts.data(), (uint32_t)wts.size()));
                    
                    Fbx::Record* transf = new Fbx::Record("Transform", cluster);
                    double id_m[16];
                    mat4 ident = mat4_identity();
                    for(int r=0; r<16; ++r) id_m[r] = (double)ident.m[r];
                    transf->properties().insert(new Fbx::Property(id_m, 16));

                    Fbx::Record* tlink = new Fbx::Record("TransformLink", cluster);
                    double m[16];
                    for(int r=0; r<16; ++r) m[r] = (double)in.world_matrices[i].m[r];
                    tlink->properties().insert(new Fbx::Property(m, 16));
                }
            }
        }
    }

    // 6.5. BindPose
    int64_t bindpose_id = generate_id();
    Fbx::Record* pose = new Fbx::Record("Pose", objs);
    pose->properties().insert(new Fbx::Property(bindpose_id));
    pose->properties().insert(new Fbx::Property("BindPose" + std::string("\x00\x01", 2) + "Pose"));
    pose->properties().insert(new Fbx::Property("BindPose"));
    (*pose->insert(new Fbx::Record("Type")))->properties().insert(new Fbx::Property("BindPose"));
    (*pose->insert(new Fbx::Record("Version")))->properties().insert(new Fbx::Property((int32_t)100));
    int32_t num_pose_nodes = (write_mesh ? (int32_t)in.meshes.size() : 0) + (int32_t)in.joints.size();
    (*pose->insert(new Fbx::Record("NbPoseNodes")))->properties().insert(new Fbx::Property(num_pose_nodes));

    auto add_pose_node = [&](int64_t node_id, const mat4& m) {
        Fbx::Record* pn = new Fbx::Record("PoseNode", pose);
        (*pn->insert(new Fbx::Record("Node")))->properties().insert(new Fbx::Property(node_id));
        Fbx::Record* pm = new Fbx::Record("Matrix", pn);
        double dm[16];
        for(int r=0; r<16; ++r) dm[r] = (double)m.m[r];
        pm->properties().insert(new Fbx::Property(dm, 16));
    };
    if (write_mesh) for (size_t mi = 0; mi < in.meshes.size(); ++mi) add_pose_node(mesh_model_ids[mi], mat4_identity());
    for (size_t i = 0; i < in.joints.size(); ++i) add_pose_node(joint_to_id[i], in.world_matrices[i]);

    // G. Animation
    struct AnimLink { int64_t stack_id = 0; int64_t layer_id = 0; 
                     struct Channel { int64_t t = 0, r = 0, s = 0; };
                     std::vector<Channel> nodes;
                     struct CurvePair { int64_t tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0, sx = 0, sy = 0, sz = 0; };
                     std::vector<CurvePair> curves; };
    std::vector<AnimLink> anim_links;

    if (write_anim) {
        for (size_t ai = 0; ai < in.animations.size(); ++ai) {
            if (anim_index >= 0 && (int)ai != anim_index) continue;
            const AnimationDef& ad = in.animations[ai];
            AnimLink link;
            link.stack_id = generate_id();
            link.layer_id = generate_id();
            Fbx::Record* stack = new Fbx::Record("AnimationStack", objs);
            stack->properties().insert(new Fbx::Property(link.stack_id));
            stack->properties().insert(new Fbx::Property(sanitize(ad.name) + std::string("\x00\x01", 2) + "AnimStack"));
            stack->properties().insert(new Fbx::Property(""));
            Fbx::Record* s_props70 = new Fbx::Record("Properties70", stack);
            double max_t = 0;
            for (const auto& ba : ad.bones) {
                if (!ba.translation.times.empty()) max_t = std::max(max_t, ba.translation.times.back());
                if (!ba.rotation.times.empty()) max_t = std::max(max_t, ba.rotation.times.back());
                if (!ba.scale.times.empty()) max_t = std::max(max_t, ba.scale.times.back());
            }
            int64_t end_time = (int64_t)std::round(max_t * KTIME_ONE_SECOND + 0.001);
            add_prop70(s_props70, "LocalStart", "KTime", "Time", "", (int64_t)0);
            add_prop70(s_props70, "LocalStop", "KTime", "Time", "", end_time);
            Fbx::Record* layer = new Fbx::Record("AnimationLayer", objs);
            layer->properties().insert(new Fbx::Property(link.layer_id));
            layer->properties().insert(new Fbx::Property(sanitize(ad.name) + std::string("\x00\x01", 2) + "AnimLayer"));
            layer->properties().insert(new Fbx::Property(""));
            link.nodes.resize(in.joints.size());
            link.curves.resize(in.joints.size());

            for (size_t ji = 0; ji < in.joints.size(); ++ji) {
                const BoneAnim& ba = ad.bones[ji];
                if (ba.translation.times.empty() && ba.rotation.times.empty() && ba.scale.times.empty()) continue;
                auto add_cn = [&](const char* name, const char* type) {
                    int64_t nid = generate_id();
                    Fbx::Record* cn = new Fbx::Record("AnimationCurveNode", objs);
                    cn->properties().insert(new Fbx::Property(nid));
                    cn->properties().insert(new Fbx::Property(std::string(name) + std::string("\x00\x01", 2) + "AnimCurveNode"));
                    cn->properties().insert(new Fbx::Property(""));
                    add_prop70(new Fbx::Record("Properties70", cn), "d", type, "", "A", (double)0);
                    return nid;
                };
                link.nodes[ji].t = add_cn("T", "Lcl Translation");
                link.nodes[ji].r = add_cn("R", "Lcl Rotation");
                link.nodes[ji].s = add_cn("S", "Lcl Scaling");
                auto add_c = [&](const std::vector<double>& times, const std::vector<double>& values) {
                    if (times.empty()) return (int64_t)0;
                    int64_t cid = generate_id();
                    Fbx::Record* c = new Fbx::Record("AnimationCurve", objs);
                    c->properties().insert(new Fbx::Property(cid));
                    c->properties().insert(new Fbx::Property(std::string("") + std::string("\x00\x01", 2) + "AnimCurve"));
                    c->properties().insert(new Fbx::Property(""));
                    std::vector<int64_t> fbx_times;
                    for (double t : times) fbx_times.push_back((int64_t)std::round(t * KTIME_ONE_SECOND + 0.001));
                    (*c->insert(new Fbx::Record("KeyTime")))->properties().insert(new Fbx::Property(fbx_times.data(), (uint32_t)fbx_times.size()));
                    std::vector<float> kvals;
                    for(double k : values) kvals.push_back((float)k);
                    (*c->insert(new Fbx::Record("KeyValueFloat")))->properties().insert(new Fbx::Property(kvals.data(), (uint32_t)kvals.size()));
                    int32_t attr_f = 4;
                    (*c->insert(new Fbx::Record("KeyAttrFlags")))->properties().insert(new Fbx::Property(&attr_f, 1));
                    float attr_d[4] = {0,0,0,0};
                    (*c->insert(new Fbx::Record("KeyAttrDataFloat")))->properties().insert(new Fbx::Property(attr_d, 4));
                    int32_t attr_r = (int32_t)times.size();
                    (*c->insert(new Fbx::Record("KeyAttrRefCount")))->properties().insert(new Fbx::Property(&attr_r, 1));
                    return cid;
                };
                if (!ba.translation.times.empty()) {
                    std::vector<double> tx, ty, tz;
                    for(size_t k=0; k<ba.translation.times.size(); ++k) { tx.push_back(ba.translation.values[k*3+0]); ty.push_back(ba.translation.values[k*3+1]); tz.push_back(ba.translation.values[k*3+2]); }
                    link.curves[ji].tx = add_c(ba.translation.times, tx); link.curves[ji].ty = add_c(ba.translation.times, ty); link.curves[ji].tz = add_c(ba.translation.times, tz);
                }
                if (!ba.rotation.times.empty()) {
                    std::vector<double> rx, ry, rz; float prev_e[3]; quat_to_euler(in.joints[ji].rotate, prev_e); float pq[4]; memcpy(pq, in.joints[ji].rotate, 16);
                    for(size_t k=0; k<ba.rotation.times.size(); ++k) {
                        double time = ba.rotation.times[k]; float t[3], q[4], s[3]; in.sample_vec3(ba.translation, time, t); in.sample_quat(ba.rotation, time, q); in.sample_vec3(ba.scale, time, s);
                        stabilize_trs(t, q, s); float dot = q[0]*pq[0]+q[1]*pq[1]+q[2]*pq[2]+q[3]*pq[3]; if(dot<0) { for(int i=0;i<4;++i) q[i]=-q[i]; } memcpy(pq,q,16);
                        float e[3]; quat_to_euler_near(q, e, prev_e); memcpy(prev_e, e, 12); rx.push_back(e[0]); ry.push_back(e[1]); rz.push_back(e[2]);
                    }
                    link.curves[ji].rx = add_c(ba.rotation.times, rx); link.curves[ji].ry = add_c(ba.rotation.times, ry); link.curves[ji].rz = add_c(ba.rotation.times, rz);
                }
                if (!ba.scale.times.empty()) {
                    std::vector<double> sx, sy, sz;
                    for(size_t k=0; k<ba.scale.times.size(); ++k) {
                        double time = ba.scale.times[k]; float t[3], q[4], s[3]; in.sample_vec3(ba.translation, time, t); in.sample_quat(ba.rotation, time, q); in.sample_vec3(ba.scale, time, s);
                        stabilize_trs(t, q, s); sx.push_back(s[0]); sy.push_back(s[1]); sz.push_back(s[2]);
                    }
                    link.curves[ji].sx = add_c(ba.scale.times, sx); link.curves[ji].sy = add_c(ba.scale.times, sy); link.curves[ji].sz = add_c(ba.scale.times, sz);
                }
            }
            anim_links.push_back(link);
        }
    }

    // 7. Connections
    Fbx::Record* conns = new Fbx::Record("Connections", &file);
    auto add_c = [&](const char* type, int64_t child, int64_t parent, const char* prop = nullptr) {
        Fbx::Record* r = new Fbx::Record("C", conns);
        r->properties().insert(new Fbx::Property(type));
        r->properties().insert(new Fbx::Property(child));
        r->properties().insert(new Fbx::Property(parent));
        if (prop) r->properties().insert(new Fbx::Property(prop));
    };

    if (write_mesh) {
        for (const auto& path : unique_textures) add_c("OO", texture_to_video_id[path], texture_to_id[path]);
        for (size_t mi = 0; mi < in.meshes.size(); ++mi) {
            add_c("OO", mesh_model_ids[mi], 0);
            add_c("OO", mesh_geom_ids[mi], mesh_model_ids[mi]);
            
            int mat_idx = in.meshes[mi].material_idx;
            if (mat_idx >= 0 && mat_idx < (int)in.materials.size()) {
                const auto& mids = material_ids[mat_idx];
                add_c("OO", mids.id, mesh_model_ids[mi]);
                
                if (mids.color_id) add_c("OP", mids.color_id, mids.id, "DiffuseColor");
                if (mids.normal_id) add_c("OP", mids.normal_id, mids.id, "NormalMap");
                if (mids.metallic_id) add_c("OP", mids.metallic_id, mids.id, "MetallicColor");
                if (mids.rough_id) add_c("OP", mids.rough_id, mids.id, "RoughnessColor");
                if (mids.emissive_id) add_c("OP", mids.emissive_id, mids.id, "EmissiveColor");
                if (mids.opacity_id) add_c("OP", mids.opacity_id, mids.id, "TransparentColor");
                if (mids.occ_id) add_c("OP", mids.occ_id, mids.id, "AmbientColor");
                if (mids.spec_id) add_c("OP", mids.spec_id, mids.id, "SpecularColor");
                if (mids.shininess_id) add_c("OP", mids.shininess_id, mids.id, "ShininessExponent");
            }

            if (!mesh_valid_cluster_ids[mi].empty()) {
                add_c("OO", skin_ids[mi], mesh_geom_ids[mi]);
                for (size_t i = 0; i < mesh_valid_cluster_ids[mi].size(); ++i) {
                    add_c("OO", mesh_valid_cluster_ids[mi][i], skin_ids[mi]);
                    add_c("OO", mesh_valid_joint_ids[mi][i], mesh_valid_cluster_ids[mi][i]);
                }
            }
        }
    }

    for (const auto& link : anim_links) {
        add_c("OO", link.stack_id, 0); add_c("OO", link.layer_id, link.stack_id);
        for (size_t ji = 0; ji < in.joints.size(); ++ji) {
            if (link.nodes[ji].t == 0) continue;
            auto add_conn = [&](int64_t nid, int64_t cx, int64_t cy, int64_t cz, const char* name) {
                add_c("OO", nid, link.layer_id);
                if (cx) add_c("OP", cx, nid, "d|X"); if (cy) add_c("OP", cy, nid, "d|Y"); if (cz) add_c("OP", cz, nid, "d|Z");
                add_c("OP", nid, joint_to_id[ji], name);
            };
            add_conn(link.nodes[ji].t, link.curves[ji].tx, link.curves[ji].ty, link.curves[ji].tz, "Lcl Translation");
            add_conn(link.nodes[ji].r, link.curves[ji].rx, link.curves[ji].ry, link.curves[ji].rz, "Lcl Rotation");
            add_conn(link.nodes[ji].s, link.curves[ji].sx, link.curves[ji].sy, link.curves[ji].sz, "Lcl Scaling");
        }
    }

    for (size_t i = 0; i < in.joints.size(); ++i) {
        if (in.joints[i].parent >= 0) add_c("OO", joint_to_id[i], joint_to_id[in.joints[i].parent]);
        else add_c("OO", joint_to_id[i], 0); 
        add_c("OO", joint_to_attr_id[i], joint_to_id[i]);
    }

    file.write(path, 7400);
    return true;
}
