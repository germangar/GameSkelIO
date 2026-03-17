#pragma once
#include <cmath>

#define ENABLE_ASSIMP 0

// Column-major 4x4 matrix (matches glTF / OpenGL convention)
// m[col*4 + row]
struct mat4 {
    float m[16];
};

inline mat4 mat4_identity() {
    mat4 r = {};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

inline void quat_normalize(float* q) {
    float len = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (len < 1e-12f) {
        q[0] = q[1] = q[2] = 0; q[3] = 1.0f;
    } else {
        float inv_len = 1.0f / len;
        q[0] *= inv_len; q[1] *= inv_len; q[2] *= inv_len; q[3] *= inv_len;
    }
}

inline void quat_mul(const float* a, const float* b, float* r) {
    float x = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    float y = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    float z = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    float w = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    r[0] = x; r[1] = y; r[2] = z; r[3] = w;
}

// Convert quaternion to Euler angles (XYZ order) in degrees
inline void quat_to_euler(const float* q, float* euler) {
    float x = q[0], y = q[1], z = q[2], w = q[3];

    // First convert to 3x3 rotation matrix
    float m[3][3];
    m[0][0] = 1.0f - 2.0f * (y*y + z*z);
    m[0][1] = 2.0f * (x*y - z*w);
    m[0][2] = 2.0f * (x*z + y*w);
    m[1][0] = 2.0f * (x*y + z*w);
    m[1][1] = 1.0f - 2.0f * (x*x + z*z);
    m[1][2] = 2.0f * (y*z - x*w);
    m[2][0] = 2.0f * (x*z - y*w);
    m[2][1] = 2.0f * (y*z + x*w);
    m[2][2] = 1.0f - 2.0f * (x*x + y*y);

    // FBX XYZ order implies R = Rz * Ry * Rx
    float sy = -m[2][0];
    if (sy >= 0.99999f) { // Gimbal lock (+90y)
        euler[1] = 90.0f;
        euler[0] = std::atan2(-m[1][2], m[1][1]) * 180.0f / 3.1415926535f;
        euler[2] = 0.0f;
    } else if (sy <= -0.99999f) { // Gimbal lock (-90y)
        euler[1] = -90.0f;
        euler[0] = std::atan2(-m[1][2], m[1][1]) * 180.0f / 3.1415926535f;
        euler[2] = 0.0f;
    } else {
        euler[1] = std::asin(sy) * 180.0f / 3.1415926535f;
        euler[0] = std::atan2(m[2][1], m[2][2]) * 180.0f / 3.1415926535f;
        euler[2] = std::atan2(m[1][0], m[0][0]) * 180.0f / 3.1415926535f;
    }
}



// Picks the solution nearest to prev_euler to avoid 180-degree flips through gimbal lock.
// This solver uses a robust "Quad-Branch" selection strategy:
// 1. Evaluate the two standard Euler branches for the orientation.
// 2. Evaluate two "gimbal-stabilized" candidates where Z is pinned to prev_euler[2].
// 3. From these 4 candidates, select the one with the smallest coordinate distance in 3D Euler space.
inline void quat_to_euler_near(const float* q, float* euler, const float* prev_euler) {
    // 1. Get the standard Euler angles for this quaternion
    float e[3];
    quat_to_euler(q, e);
    
    // 2. An Euler rotation (X, Y, Z) has a mathematical alias:
    //    (X + 180, 180 - Y, Z + 180) modulo 360
    float alias[3];
    alias[0] = e[0] + 180.0f;
    alias[1] = 180.0f - e[1];
    alias[2] = e[2] + 180.0f;

    // Helper to wrap a value to the nearest multiple of 360 around a target
    auto wrap_near = [](float val, float target) {
        float diff = val - target;
        while (diff > 180.0f) { val -= 360.0f; diff -= 360.0f; }
        while (diff < -180.0f) { val += 360.0f; diff += 360.0f; }
        return val;
    };

    // Helper for squared Euclidean distance
    auto d2 = [](const float* a, const float* p) {
        float dx = a[0] - p[0];
        float dy = a[1] - p[1];
        float dz = a[2] - p[2];
        return dx*dx + dy*dy + dz*dz;
    };

    // Wrap both the standard angles and the alias near the previous frame's angles
    float c1[3] = { wrap_near(e[0], prev_euler[0]), wrap_near(e[1], prev_euler[1]), wrap_near(e[2], prev_euler[2]) };
    float c2[3] = { wrap_near(alias[0], prev_euler[0]), wrap_near(alias[1], prev_euler[1]), wrap_near(alias[2], prev_euler[2]) };

    // Choose the one that minimizes the jump from the previous frame
    if (d2(c1, prev_euler) < d2(c2, prev_euler)) {
        euler[0] = c1[0]; euler[1] = c1[1]; euler[2] = c1[2];
    } else {
        euler[0] = c2[0]; euler[1] = c2[1]; euler[2] = c2[2];
    }
}


// Factor out negative scales into the rotation to prevent visual flipping during interpolation.
// Only works if we have an even number of negative scales (pure rotation).
// If we have an odd number (reflection), we can at most move it to a single axis.
inline void stabilize_trs(float* t, float* q, float* s) {
    bool flip_x = s[0] < 0;
    bool flip_y = s[1] < 0;
    bool flip_z = s[2] < 0;
    
    // Handle even flips (pure rotations)
    if (flip_x && flip_y) { s[0] = -s[0]; s[1] = -s[1]; float r[4] = {0,0,1,0}; float nq[4]; quat_mul(q, r, nq); memcpy(q, nq, 16); flip_x = flip_y = false; }
    if (flip_x && flip_z) { s[0] = -s[0]; s[2] = -s[2]; float r[4] = {0,1,0,0}; float nq[4]; quat_mul(q, r, nq); memcpy(q, nq, 16); flip_x = flip_z = false; }
    if (flip_y && flip_z) { s[1] = -s[1]; s[2] = -s[2]; float r[4] = {1,0,0,0}; float nq[4]; quat_mul(q, r, nq); memcpy(q, nq, 16); flip_y = flip_z = false; }
    
    quat_normalize(q);
}

inline void quat_rotate_vec(const float* q, const float* v, float* r) {
    float t[3];
    t[0] = 2.0f * (q[1] * v[2] - q[2] * v[1]);
    t[1] = 2.0f * (q[2] * v[0] - q[0] * v[2]);
    t[2] = 2.0f * (q[0] * v[1] - q[1] * v[0]);
    r[0] = v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]);
    r[1] = v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]);
    r[2] = v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0]);
}

inline mat4 mat4_mul(const mat4& a, const mat4& b) {
    mat4 r;
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++) sum += a.m[k*4 + row] * b.m[col*4 + k];
            r.m[col*4 + row] = sum;
        }
    }
    return r;
}

// Build a mat4 from TRS (translate, rotate[xyzw], scale)
// Safely handles NULL for t (defaults to 0) or s (defaults to 1)
inline mat4 mat4_from_trs(const float* t, const float* r, const float* s) {
    mat4 m = mat4_identity();
    float tx = t ? t[0] : 0, ty = t ? t[1] : 0, tz = t ? t[2] : 0;
    float sx = s ? s[0] : 1, sy = s ? s[1] : 1, sz = s ? s[2] : 1;
    float x = 0, y = 0, z = 0, w = 1;
    if (r) { x = r[0]; y = r[1]; z = r[2]; w = r[3]; }
    m.m[0]  = (1 - 2*y*y - 2*z*z) * sx;
    m.m[1]  = (2*x*y + 2*z*w)      * sx;
    m.m[2]  = (2*x*z - 2*y*w)      * sx;
    m.m[4]  = (2*x*y - 2*z*w)      * sy;
    m.m[5]  = (1 - 2*x*x - 2*z*z)  * sy;
    m.m[6]  = (2*y*z + 2*x*w)      * sy;
    m.m[8]  = (2*x*z + 2*y*w)      * sz;
    m.m[9]  = (2*y*z - 2*x*w)      * sz;
    m.m[10] = (1 - 2*x*x - 2*y*y)  * sz;
    m.m[12] = tx; m.m[13] = ty; m.m[14] = tz;
    return m;
}

// General 4x4 invert. Forces affine integrity on result.
inline mat4 mat4_invert(const mat4& m) {
    mat4 inv;
    float s0 = m.m[0]*m.m[5]  - m.m[4]*m.m[1];
    float s1 = m.m[0]*m.m[6]  - m.m[4]*m.m[2];
    float s2 = m.m[0]*m.m[7]  - m.m[4]*m.m[3];
    float s3 = m.m[1]*m.m[6]  - m.m[5]*m.m[2];
    float s4 = m.m[1]*m.m[7]  - m.m[5]*m.m[3];
    float s5 = m.m[2]*m.m[7]  - m.m[6]*m.m[3];
    float c5 = m.m[10]*m.m[15] - m.m[14]*m.m[11];
    float c4 = m.m[9]*m.m[15]  - m.m[13]*m.m[11];
    float c3 = m.m[9]*m.m[14]  - m.m[13]*m.m[10];
    float c2 = m.m[8]*m.m[15]  - m.m[12]*m.m[11];
    float c1 = m.m[8]*m.m[14]  - m.m[12]*m.m[10];
    float c0 = m.m[8]*m.m[13]  - m.m[12]*m.m[9];
    float det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    if (std::abs(det) < 1e-6f) return mat4_identity();
    float idet = 1.0f / det;
    inv.m[0]  = ( m.m[5]*c5  - m.m[6]*c4  + m.m[7]*c3)  * idet;
    inv.m[1]  = (-m.m[1]*c5  + m.m[2]*c4  - m.m[3]*c3)  * idet;
    inv.m[2]  = ( m.m[13]*s5 - m.m[14]*s4 + m.m[15]*s3) * idet;
    inv.m[3]  = (-m.m[9]*s5  + m.m[10]*s4 - m.m[11]*s3) * idet;
    inv.m[4]  = (-m.m[4]*c5  + m.m[6]*c2  - m.m[7]*c1)  * idet;
    inv.m[5]  = ( m.m[0]*c5  - m.m[2]*c2  + m.m[3]*c1)  * idet;
    inv.m[6]  = (-m.m[12]*s5 + m.m[14]*s2 - m.m[15]*s1) * idet;
    inv.m[7]  = ( m.m[8]*s5  - m.m[10]*s2 + m.m[11]*s1) * idet;
    inv.m[8]  = ( m.m[4]*c4  - m.m[5]*c2  + m.m[7]*c0)  * idet;
    inv.m[9]  = (-m.m[0]*c4  + m.m[1]*c2  - m.m[3]*c0)  * idet;
    inv.m[10] = ( m.m[12]*s4 - m.m[13]*s2 + m.m[15]*s0) * idet;
    inv.m[11] = (-m.m[8]*s4  + m.m[9]*s2  - m.m[11]*s0) * idet;
    inv.m[12] = (-m.m[4]*c3  + m.m[5]*c1  - m.m[6]*c0)  * idet;
    inv.m[13] = ( m.m[0]*c3  - m.m[1]*c1  + m.m[2]*c0)  * idet;
    inv.m[14] = (-m.m[12]*s3 + m.m[13]*s1 - m.m[14]*s0) * idet;
    inv.m[15] = ( m.m[8]*s3  - m.m[9]*s1  + m.m[10]*s0) * idet;
    // Force affine integrity (no perspective component)
    inv.m[3] = inv.m[7] = inv.m[11] = 0.0f;
    inv.m[15] = 1.0f;
    return inv;
}

inline mat4 mat4_transpose(const mat4& m) {
    mat4 r;
    r.m[0] = m.m[0]; r.m[1] = m.m[4]; r.m[2] = m.m[8]; r.m[3] = m.m[12];
    r.m[4] = m.m[1]; r.m[5] = m.m[5]; r.m[6] = m.m[9]; r.m[7] = m.m[13];
    r.m[8] = m.m[2]; r.m[9] = m.m[6]; r.m[10] = m.m[10]; r.m[11] = m.m[14];
    r.m[12] = m.m[3]; r.m[13] = m.m[7]; r.m[14] = m.m[11]; r.m[15] = m.m[15];
    return r;
}

// Decompose a column-major matrix into TRS components.
// Only works for valid TRS matrices (no skew/perspective).
inline void mat4_decompose(const mat4& m, float* t, float* q, float* s) {
    if (t) { t[0] = m.m[12]; t[1] = m.m[13]; t[2] = m.m[14]; }
    
    // Extract scale as column lengths
    float sx = sqrtf(m.m[0]*m.m[0] + m.m[1]*m.m[1] + m.m[2]*m.m[2]);
    float sy = sqrtf(m.m[4]*m.m[4] + m.m[5]*m.m[5] + m.m[6]*m.m[6]);
    float sz = sqrtf(m.m[8]*m.m[8] + m.m[9]*m.m[9] + m.m[10]*m.m[10]);
    
    // Determine if the matrix has a reflection by checking the determinant of the rotation part
    // Using a simplified determinant check on the unnormalized columns
    float det = m.m[0]*(m.m[5]*m.m[10] - m.m[6]*m.m[9]) - m.m[4]*(m.m[1]*m.m[10] - m.m[2]*m.m[9]) + m.m[8]*(m.m[1]*m.m[6] - m.m[2]*m.m[5]);
    if (det < 0) sx = -sx;

    if (s) { s[0] = sx; s[1] = sy; s[2] = sz; }
    
    if (q) {
        mat4 r = m;
        if (std::abs(sx) > 1e-6f) { r.m[0] /= sx; r.m[1] /= sx; r.m[2] /= sx; }
        if (std::abs(sy) > 1e-6f) { r.m[4] /= sy; r.m[5] /= sy; r.m[6] /= sy; }
        if (std::abs(sz) > 1e-6f) { r.m[8] /= sz; r.m[9] /= sz; r.m[10] /= sz; }
        
        float tr = r.m[0] + r.m[5] + r.m[10];
        if (tr > 0) {
            float s_val = sqrtf(tr + 1.0f) * 2;
            q[3] = 0.25f * s_val;
            q[0] = (r.m[6] - r.m[9]) / s_val;
            q[1] = (r.m[8] - r.m[2]) / s_val;
            q[2] = (r.m[1] - r.m[4]) / s_val;
        } else if ((r.m[0] > r.m[5]) && (r.m[0] > r.m[10])) {
            float s_val = sqrtf(1.0f + r.m[0] - r.m[5] - r.m[10]) * 2;
            q[3] = (r.m[6] - r.m[9]) / s_val;
            q[0] = 0.25f * s_val;
            q[1] = (r.m[4] + r.m[1]) / s_val;
            q[2] = (r.m[2] + r.m[8]) / s_val;
        } else if (r.m[5] > r.m[10]) {
            float s_val = sqrtf(1.0f + r.m[5] - r.m[0] - r.m[10]) * 2;
            q[3] = (r.m[8] - r.m[2]) / s_val;
            q[0] = (r.m[4] + r.m[1]) / s_val;
            q[1] = 0.25f * s_val;
            q[2] = (r.m[9] + r.m[6]) / s_val;
        } else {
            float s_val = sqrtf(1.0f + r.m[10] - r.m[0] - r.m[5]) * 2;
            q[3] = (r.m[1] - r.m[4]) / s_val;
            q[0] = (r.m[2] + r.m[8]) / s_val;
            q[1] = (r.m[9] + r.m[6]) / s_val;
            q[2] = 0.25f * s_val;
        }
        quat_normalize(q);
    }
}

inline void mat4_mul_vec3(const mat4& m, const float* v, float* r) {
    float x = v[0], y = v[1], z = v[2];
    r[0] = m.m[0]*x + m.m[4]*y + m.m[8]*z + m.m[12];
    r[1] = m.m[1]*x + m.m[5]*y + m.m[9]*z + m.m[13];
    r[2] = m.m[2]*x + m.m[6]*y + m.m[10]*z + m.m[14];
}
