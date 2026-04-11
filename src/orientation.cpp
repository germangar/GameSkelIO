#include "orientation.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <utility>

struct Basis {
    float r[3]; // Right
    float u[3]; // Up
    float f[3]; // Forward
};

static Basis get_basis(gs_coord_system sys) {
    switch (sys) {
        // Right-Handed (det = +1)
        case GS_Y_UP_RIGHTHANDED:       return {{ 1, 0, 0}, { 0, 1, 0}, { 0, 0, 1}}; // glTF (+Z Forward)
        case GS_Z_UP_RIGHTHANDED:       return {{ 1, 0, 0}, { 0, 0, 1}, { 0,-1, 0}}; // Blender (-Y Forward)
        case GS_Z_UP_RIGHTHANDED_X_FWD: return {{ 0, 1, 0}, { 0, 0, 1}, { 1, 0, 0}}; // IQM (+X Forward)
        case GS_X_UP_RIGHTHANDED:       return {{ 0,-1, 0}, { 1, 0, 0}, { 0, 0, 1}}; // (+Z Forward)

        // Left-Handed (det = -1)
        case GS_Y_UP_LEFTHANDED:        return {{ 1, 0, 0}, { 0, 1, 0}, { 0, 0,-1}}; // Unity (-Z Forward)
        case GS_Z_UP_LEFTHANDED:        return {{ 0,-1, 0}, { 0, 0, 1}, { 1, 0, 0}}; // Unreal (+X Forward)
        case GS_X_UP_LEFTHANDED:        return {{ 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1}}; // (+Z Forward)
    }
    return {{1,0,0}, {0,1,0}, {0,0,1}};
}

static mat4 compute_conversion_matrix(gs_coord_system src_sys, gs_coord_system dst_sys) {
    Basis src = get_basis(src_sys);
    Basis dst = get_basis(dst_sys);

    // We want to find M such that M * src_basis = dst_basis
    auto build_mat = [](Basis b) {
        mat4 r = mat4_identity();
        r.m[0] = b.r[0]; r.m[1] = b.r[1]; r.m[2] = b.r[2];
        r.m[4] = b.u[0]; r.m[5] = b.u[1]; r.m[6] = b.u[2];
        r.m[8] = b.f[0]; r.m[9] = b.f[1]; r.m[10] = b.f[2];
        return r;
    };

    mat4 S = build_mat(src);
    mat4 D = build_mat(dst);
    mat4 S_inv = mat4_invert(S);

    return mat4_mul(D, S_inv);
}

static float mat4_det3x3(const mat4& m) {
    return m.m[0] * (m.m[5] * m.m[10] - m.m[6] * m.m[9]) -
           m.m[4] * (m.m[1] * m.m[10] - m.m[2] * m.m[9]) +
           m.m[8] * (m.m[1] * m.m[6] - m.m[2] * m.m[5]);
}

bool convert_orientation(Model& model, gs_coord_system target, gs_winding_order target_winding) {
    if (model.orientation == target && model.winding == target_winding) return true;

    // 1. Handle Orientation Change
    if (model.orientation != target) {
        mat4 M = compute_conversion_matrix(model.orientation, target);
        mat4 M_inv = mat4_invert(M);
        mat4 M_rot = M;
        M_rot.m[12] = M_rot.m[13] = M_rot.m[14] = 0.0f;

        float det = mat4_det3x3(M);
        bool flips_handedness = (det < 0);

        if (flips_handedness) {
            model.winding = (model.winding == GS_WINDING_CCW) ? GS_WINDING_CW : GS_WINDING_CCW;
        }

        // Vertices
        for (size_t i = 0; i < model.positions.size() / 3; ++i) {
            float p[3] = { model.positions[i*3], model.positions[i*3+1], model.positions[i*3+2] };
            float p_new[3];
            mat4_mul_vec3(M, p, p_new);
            model.positions[i*3] = p_new[0];
            model.positions[i*3+1] = p_new[1];
            model.positions[i*3+2] = p_new[2];
        }

        // Normals
        if (!model.normals.empty()) {
            for (size_t i = 0; i < model.normals.size() / 3; ++i) {
                float n[3] = { model.normals[i*3], model.normals[i*3+1], model.normals[i*3+2] };
                float n_new[3];
                mat4_mul_vec3(M_rot, n, n_new);
                float len = sqrtf(n_new[0]*n_new[0] + n_new[1]*n_new[1] + n_new[2]*n_new[2]);
                if (len > 1e-6f) {
                    model.normals[i*3] = n_new[0] / len;
                    model.normals[i*3+1] = n_new[1] / len;
                    model.normals[i*3+2] = n_new[2] / len;
                }
            }
        }

        // Tangents
        if (!model.tangents.empty()) {
            float det_sign = flips_handedness ? -1.0f : 1.0f;
            for (size_t i = 0; i < model.tangents.size() / 4; ++i) {
                float t[3] = { model.tangents[i*4], model.tangents[i*4+1], model.tangents[i*4+2] };
                float t_new[3];
                mat4_mul_vec3(M_rot, t, t_new);
                float len = sqrtf(t_new[0]*t_new[0] + t_new[1]*t_new[1] + t_new[2]*t_new[2]);
                if (len > 1e-6f) {
                    model.tangents[i*4] = t_new[0] / len;
                    model.tangents[i*4+1] = t_new[1] / len;
                    model.tangents[i*4+2] = t_new[2] / len;
                }
                model.tangents[i*4+3] *= det_sign;
            }
        }

        // Morph Targets
        for (auto& mt : model.morph_targets) {
            if (!mt.positions.empty()) {
                for (size_t i = 0; i < mt.positions.size() / 3; ++i) {
                    float p[3] = { mt.positions[i*3], mt.positions[i*3+1], mt.positions[i*3+2] };
                    float p_new[3];
                    mat4_mul_vec3(M_rot, p, p_new);
                    mt.positions[i*3] = p_new[0];
                    mt.positions[i*3+1] = p_new[1];
                    mt.positions[i*3+2] = p_new[2];
                }
            }
            if (!mt.normals.empty()) {
                for (size_t i = 0; i < mt.normals.size() / 3; ++i) {
                    float n[3] = { mt.normals[i*3], mt.normals[i*3+1], mt.normals[i*3+2] };
                    float n_new[3];
                    mat4_mul_vec3(M_rot, n, n_new);
                    mt.normals[i*3] = n_new[0];
                    mt.normals[i*3+1] = n_new[1];
                    mt.normals[i*3+2] = n_new[2];
                }
            }
        }

        // Skeleton
        for (size_t i = 0; i < model.joints.size(); ++i) {
            if (model.joints[i].parent == -1) {
                auto& joint = model.joints[i];
                mat4 L = mat4_from_trs(joint.translate, joint.rotate, joint.scale);
                mat4 L_new = mat4_mul(M, L);
                mat4_decompose(L_new, joint.translate, joint.rotate, joint.scale);

                for (auto& anim : model.animations) {
                    auto& bone = anim.bones[i];
                    for (size_t k = 0; k < bone.translation.values.size() / 3; ++k) {
                        float t[3] = { bone.translation.values[k*3], bone.translation.values[k*3+1], bone.translation.values[k*3+2] };
                        float t_new[3];
                        mat4_mul_vec3(M, t, t_new);
                        bone.translation.values[k*3] = t_new[0];
                        bone.translation.values[k*3+1] = t_new[1];
                        bone.translation.values[k*3+2] = t_new[2];
                    }
                    float prev_q[4] = {0,0,0,1};
                    for (size_t k = 0; k < bone.rotation.values.size() / 4; ++k) {
                        float q_A[4] = { bone.rotation.values[k*4], bone.rotation.values[k*4+1], bone.rotation.values[k*4+2], bone.rotation.values[k*4+3] };
                        mat4 R_A = mat4_from_trs(nullptr, q_A, nullptr);
                        mat4 R_new = mat4_mul(M_rot, R_A);
                        float q_new[4];
                        mat4_decompose(R_new, nullptr, q_new, nullptr);
                        if (k > 0) {
                            float dot = q_new[0]*prev_q[0] + q_new[1]*prev_q[1] + q_new[2]*prev_q[2] + q_new[3]*prev_q[3];
                            if (dot < 0) { q_new[0]=-q_new[0]; q_new[1]=-q_new[1]; q_new[2]=-q_new[2]; q_new[3]=-q_new[3]; }
                        }
                        memcpy(&bone.rotation.values[k*4], q_new, 16);
                        memcpy(prev_q, q_new, 16);
                    }
                }
            }
        }

        if (!model.ibms.empty()) {
            for (auto& ibm : model.ibms) ibm = mat4_mul(ibm, M_inv);
        }

        model.orientation = target;
    }

    // 2. Ensure Winding
    if (model.winding != target_winding && !model.indices.empty()) {
        for (size_t i = 0; i < model.indices.size(); i += 3) {
            std::swap(model.indices[i + 1], model.indices[i + 2]);
        }
        model.winding = target_winding;
    }

    // 3. Finalize
    model.compute_bind_pose();
    if (model.has_bounds) model.compute_bounds();

    return true;
}
