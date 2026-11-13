// Regression test for the MD3 tag-only importer (gsk_load_md3_tags).
//
// Goal: prove that a tag-only MD3, exported to IQM, reproduces the MD3 tag
// orientation and position through Warfork's *skeletal* attachment path.
//
// Warfork specifics that this test mirrors (source/warfork-qfusion):
//
//   ref_gl/r_alias.c      : MD3 tags are stored as forward/right/up rows.
//                           The rows are copied into a row-major mat3_t and
//                           converted with Quat_FromMatrix3(). R_AliasModelLerpTag()
//                           exposes that quaternion as the tag orientation.
//   ref_gl/r_skm.c        : IQM joints/frames are converted to dual quaternions.
//                           Frame rotations with w > 0 are negated.
//   cgame/cg_boneposes.cpp : CG_TransformBoneposes() multiplies the parent
//                           dual quaternion with the child one.
//                           CG_SkeletalPoseGetAttachment() returns the INVERSE
//                           of the absolute bone rotation plus the absolute
//                           bone translation.
//
// Because the client inverts the absolute bone rotation, the imported tag
// joint must carry the inverse of the MD3 tag rotation for the final
// attachment to match the original MD3 runtime behaviour.

#include "gameskelio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// tiny test harness
// ---------------------------------------------------------------------------
static int g_failures = 0;

static void check(bool ok, const char* what) {
    if (!ok) {
        printf("  FAIL: %s\n", what);
        ++g_failures;
    }
}

static void check_close(float a, float b, float tol, const char* what) {
    if (!(std::fabs(a - b) <= tol)) {
        printf("  FAIL: %s (%.9g vs %.9g, tol %.3g)\n", what, a, b, tol);
        ++g_failures;
    }
}

// ---------------------------------------------------------------------------
// quaternion / matrix helpers (mirroring Warfork's q_math.c)
// ---------------------------------------------------------------------------
static void q_mul(const float a[4], const float b[4], float out[4]) {
    out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

static void q_conj(const float in[4], float out[4]) {
    out[0] = -in[0]; out[1] = -in[1]; out[2] = -in[2]; out[3] = in[3];
}

static void q_normalize(float q[4]) {
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len < 1e-12f) { q[0] = q[1] = q[2] = 0.0f; q[3] = 1.0f; return; }
    float inv = 1.0f / len;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

static void q_rotate(const float q[4], const float v[3], float out[3]) {
    float t[3];
    t[0] = 2.0f * (q[1] * v[2] - q[2] * v[1]);
    t[1] = 2.0f * (q[2] * v[0] - q[0] * v[2]);
    t[2] = 2.0f * (q[0] * v[1] - q[1] * v[0]);
    out[0] = v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]);
    out[1] = v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]);
    out[2] = v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0]);
}

// Warfork Quat_ToMatrix3(): writes forward (m[0..2]), right (m[3..5]) and
// up (m[6..8]) as consecutive rows.
static void q_to_matrix3(const float q[4], float m[9]) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    m[0] = 1.0f - 2.0f * (yy + zz); m[1] = 2.0f * (xy - wz);       m[2] = 2.0f * (xz + wy);
    m[3] = 2.0f * (xy + wz);       m[4] = 1.0f - 2.0f * (xx + zz); m[5] = 2.0f * (yz - wx);
    m[6] = 2.0f * (xz - wy);       m[7] = 2.0f * (yz + wx);       m[8] = 1.0f - 2.0f * (xx + yy);
}

// ---------------------------------------------------------------------------
// synthetic tags-only MD3 writer
// ---------------------------------------------------------------------------
static void put_u32(std::vector<uint8_t>& f, uint32_t v) {
    f.push_back((uint8_t)(v & 0xFF)); f.push_back((uint8_t)((v >> 8) & 0xFF));
    f.push_back((uint8_t)((v >> 16) & 0xFF)); f.push_back((uint8_t)((v >> 24) & 0xFF));
}
static void put_f32(std::vector<uint8_t>& f, float v) {
    uint32_t u; memcpy(&u, &v, 4); put_u32(f, u);
}
static void put_tag(std::vector<uint8_t>& f, const char* name,
                    const float origin[3], const float axis[9]) {
    char nm[64]; memset(nm, 0, 64); strncpy(nm, name, 63);
    f.insert(f.end(), (uint8_t*)nm, (uint8_t*)nm + 64);
    for (int i = 0; i < 3; ++i) put_f32(f, origin[i]);
    for (int i = 0; i < 9; ++i) put_f32(f, axis[i]);
}

struct TagSample {
    float origin[3];
    float axis[9]; // forward row, right row, up row (MD3 storage order)
};

// Build a proper orthonormal basis (rows) by rotating the identity matrix.
static void make_basis(float angleDeg, float axisOut[9]) {
    const float rad = angleDeg * 3.14159265358979f / 180.0f;
    const float c = std::cos(rad), s = std::sin(rad);
    // rotation about Z: forward = (c, s, 0), right = (-s, c, 0), up = (0, 0, 1)
    axisOut[0] = c;  axisOut[1] = s;  axisOut[2] = 0.0f;
    axisOut[3] = -s; axisOut[4] = c;  axisOut[5] = 0.0f;
    axisOut[6] = 0.0f; axisOut[7] = 0.0f; axisOut[8] = 1.0f;
}

static bool write_synthetic_md3(const std::string& path, const std::vector<TagSample>& frames) {
    const uint32_t FRAMES = (uint32_t)frames.size();
    const uint32_t TAGS = 1;

    std::vector<uint8_t> f;
    f.insert(f.end(), { 'I', 'D', 'P', '3' });
    put_u32(f, 15);
    char name[64]; memset(name, 0, 64); strncpy(name, "tags_only.md3", 63);
    f.insert(f.end(), (uint8_t*)name, (uint8_t*)name + 64);
    put_u32(f, 0);                                       // flags
    put_u32(f, FRAMES);                                  // num_frames
    put_u32(f, TAGS);                                    // num_tags
    put_u32(f, 0);                                       // num_surfaces (tags-only!)
    put_u32(f, 0);                                       // num_skins
    const uint32_t ofs_tags = 108 + FRAMES * 56;
    put_u32(f, 108);                                     // ofs_frames
    put_u32(f, ofs_tags);                                // ofs_tags
    put_u32(f, ofs_tags + FRAMES * TAGS * 112);          // ofs_surfaces
    put_u32(f, ofs_tags + FRAMES * TAGS * 112);          // ofs_eof

    for (uint32_t i = 0; i < FRAMES; ++i) {              // frame headers
        for (int k = 0; k < 3; ++k) put_f32(f, 0.0f);    // mins
        for (int k = 0; k < 3; ++k) put_f32(f, 0.0f);    // maxs
        for (int k = 0; k < 3; ++k) put_f32(f, 0.0f);    // origin
        put_f32(f, 0.0f);                                // radius
        char fn[16]; memset(fn, 0, 16); strncpy(fn, "frame", 15);
        f.insert(f.end(), (uint8_t*)fn, (uint8_t*)fn + 16);
    }
    for (uint32_t fi = 0; fi < FRAMES; ++fi) {
        put_tag(f, "tag_weapon", frames[fi].origin, frames[fi].axis);
    }

    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) return false;
    fwrite(f.data(), 1, f.size(), fp);
    fclose(fp);
    return true;
}

// ---------------------------------------------------------------------------
// minimal IQM reader (enough for joints + frame channels)
// ---------------------------------------------------------------------------
struct IqmJoint {
    std::string name;
    int parent;
    float translate[3];
    float rotate[4];
};
struct IqmPose {
    int parent;
    uint32_t mask;
    float offset[10];
    float scale[10];
};
struct IqmFile {
    std::vector<IqmJoint> joints;
    std::vector<IqmPose> poses;
    uint32_t num_anims = 0;
    uint32_t num_frames = 0;
    std::vector<uint16_t> frame_data; // num_frames * num_poses * 10
};

static bool read_iqm(const std::string& path, IqmFile& out) {
    FILE* fp = fopen(path.c_str(), "rb");
    if (!fp) return false;
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    std::vector<uint8_t> data((size_t)size);
    if (fread(data.data(), 1, (size_t)size, fp) != (size_t)size) { fclose(fp); return false; }
    fclose(fp);

    if (data.size() < 16 + 27 * 4) return false;
    if (memcmp(data.data(), "INTERQUAKEMODEL", 16) != 0) return false;

    auto u32 = [&](size_t i) -> uint32_t {
        uint32_t v; memcpy(&v, data.data() + 16 + i * 4, 4); return v;
    };
    // index map (after magic): 0 version, 1 filesize, 2 flags, then pairs
    const uint32_t ofs_text = u32(4);
    const uint32_t num_joints = u32(13), ofs_joints = u32(14);
    const uint32_t num_poses = u32(15), ofs_poses = u32(16);
    const uint32_t num_anims = u32(17);
    const uint32_t num_frames = u32(19);
    const uint32_t num_channels = u32(20), ofs_frames = u32(21);

    const char* text = (const char*)data.data() + ofs_text;
    for (uint32_t i = 0; i < num_joints; ++i) {
        const uint8_t* p = data.data() + ofs_joints + (size_t)i * 48;
        uint32_t noff; memcpy(&noff, p, 4);
        int parent; memcpy(&parent, p + 4, 4);
        IqmJoint j;
        j.name = std::string(text + noff);
        j.parent = parent;
        memcpy(j.translate, p + 8, 12);
        memcpy(j.rotate, p + 20, 16);
        out.joints.push_back(j);
    }
    for (uint32_t i = 0; i < num_poses; ++i) {
        const uint8_t* p = data.data() + ofs_poses + (size_t)i * 88;
        IqmPose pose;
        memcpy(&pose.parent, p, 4);
        memcpy(&pose.mask, p + 4, 4);
        memcpy(pose.offset, p + 8, 40);
        memcpy(pose.scale, p + 48, 40);
        out.poses.push_back(pose);
    }
    out.num_frames = num_frames;
    out.num_anims = num_anims;
    out.frame_data.resize((size_t)num_frames * num_channels);
    if (!out.frame_data.empty()) {
        memcpy(out.frame_data.data(), data.data() + ofs_frames,
               out.frame_data.size() * sizeof(uint16_t));
    }
    return true;
}

// Warfork ref_gl/r_skm.c frame reconstruction for one pose.
static void iqm_pose_channels(const IqmFile& iqm, uint32_t frame, uint32_t pose,
                              float values[10]) {
    const IqmPose& p = iqm.poses[pose];
    const size_t base = ((size_t)frame * iqm.poses.size() + pose) * 10;
    for (int c = 0; c < 10; ++c) values[c] = p.offset[c];
    for (int c = 0; c < 10; ++c) {
        if (!(p.mask & (1u << c))) continue;
        values[c] += (float)iqm.frame_data[base + c] * p.scale[c];
    }
}

// ---------------------------------------------------------------------------
// test
// ---------------------------------------------------------------------------
int main() {
    // Write the scratch files into the system temp directory so the test
    // never pollutes the source tree.
    std::string tmpdir;
    if (const char* t = getenv("TEMP")) tmpdir = t;
    else if (const char* t = getenv("TMP")) tmpdir = t;
    if (!tmpdir.empty() && tmpdir.back() != '/' && tmpdir.back() != '\\') tmpdir += '/';
    const std::string md3_path = tmpdir + "test_md3_tags_input.md3";
    const std::string iqm_path = tmpdir + "test_md3_tags_output.iqm";

    // Three tag frames: identity, a 40 degree turn, a 100 degree turn.
    std::vector<TagSample> frames(3);
    frames[0].origin[0] =  0.0f; frames[0].origin[1] =  0.0f; frames[0].origin[2] = 0.0f;
    make_basis(0.0f, frames[0].axis);
    frames[1].origin[0] = 10.0f; frames[1].origin[1] = -3.5f; frames[1].origin[2] = 7.25f;
    make_basis(40.0f, frames[1].axis);
    frames[2].origin[0] = -4.5f; frames[2].origin[1] = 12.0f; frames[2].origin[2] = -11.40625f;
    make_basis(100.0f, frames[2].axis);

    if (!write_synthetic_md3(md3_path, frames)) {
        printf("FAILED to write synthetic MD3\n");
        return 1;
    }

    printf("== load tag-only MD3 ==\n");
    gs_model* m = gsk_load_md3_tags(md3_path.c_str());
    check(m != nullptr, "gsk_load_md3_tags() succeeds");
    if (!m) return 1;

    check(m->num_joints == 2, "two joints (synthetic root + tag)");
    check(m->num_meshes == 0, "no meshes");
    check(m->num_animations == 1, "one animation");
    check(m->orientation == GS_Z_UP_RIGHTHANDED_X_FWD, "source orientation is Z-up +X forward");

    if (m->num_joints == 2) {
        check(m->joints[1].parent == 0, "tag joint is a child of the root");
        check(std::string(m->joints[1].name) == "tag_weapon", "tag joint keeps its MD3 name");
        check_close(m->joints[1].translate[0], frames[0].origin[0], 1e-6f, "tag origin x");
        check_close(m->joints[1].translate[1], frames[0].origin[1], 1e-6f, "tag origin y");
        check_close(m->joints[1].translate[2], frames[0].origin[2], 1e-6f, "tag origin z");

        // The imported joint rotation must be the INVERSE of the MD3 basis.
        float md3_matrix[9];
        for (int i = 0; i < 9; ++i) md3_matrix[i] = frames[0].axis[i];
        float imp_matrix[9];
        {
            // rotation matrix of the imported quaternion, column-major
            const float q[4] = { m->joints[1].rotate[0], m->joints[1].rotate[1],
                                 m->joints[1].rotate[2], m->joints[1].rotate[3] };
            float rows[9]; q_to_matrix3(q, rows);
            // rows are forward/right/up; transpose them to compare with D^T
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    imp_matrix[c * 3 + r] = rows[r * 3 + c];
        }
        double max_err = 0.0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                max_err = std::max(max_err, (double)std::fabs(imp_matrix[r * 3 + c] - md3_matrix[c * 3 + r]));
        check(max_err < 1e-5, "imported joint rotation is the inverse MD3 basis");
    }

    // gskelview converts to Y-up on load; the exporter converts back.
    gsk_convert_orientation(m, GS_Y_UP_RIGHTHANDED, GS_WINDING_CCW);
    check(m->orientation == GS_Y_UP_RIGHTHANDED, "model converted to Y-up");

    printf("== export to IQM ==\n");
    check(gsk_write_iqm(iqm_path.c_str(), m, false), "gsk_write_iqm() succeeds");
    gsk_free_model(m);

    printf("== simulate Warfork IQM attachment ==\n");
    IqmFile iqm;
    check(read_iqm(iqm_path, iqm), "IQM file parsed");
    check(iqm.joints.size() == 2, "IQM has two joints");
    check(iqm.num_frames == frames.size(), "IQM frame count matches the MD3 frame count");

    if (iqm.joints.size() == 2) {
        // Locate the tag joint by name, exactly like CG_SkeletalPoseGetAttachment().
        int tag_index = -1;
        for (size_t i = 0; i < iqm.joints.size(); ++i)
            if (iqm.joints[i].name == "tag_weapon") tag_index = (int)i;
        check(tag_index >= 0, "IQM contains a joint named tag_weapon");

        if (tag_index >= 0) {
            for (uint32_t f = 0; f < iqm.num_frames && f < frames.size(); ++f) {
                // Local pose of the tag joint for this frame.
                float values[10];
                iqm_pose_channels(iqm, f, (uint32_t)tag_index, values);
                float local_t[3] = { values[0], values[1], values[2] };
                float local_q[4] = { values[3], values[4], values[5], values[6] };
                // r_skm.c: frame rotations with w > 0 are negated.
                if (local_q[3] > 0.0f) {
                    local_q[0] = -local_q[0]; local_q[1] = -local_q[1];
                    local_q[2] = -local_q[2]; local_q[3] = -local_q[3];
                }
                q_normalize(local_q);

                // Absolute pose: walk the parent chain (CG_TransformBoneposes).
                float abs_q[4] = { local_q[0], local_q[1], local_q[2], local_q[3] };
                float abs_t[3] = { local_t[0], local_t[1], local_t[2] };
                int parent = iqm.joints[tag_index].parent;
                while (parent >= 0) {
                    float pv[10];
                    iqm_pose_channels(iqm, f, (uint32_t)parent, pv);
                    float pq[4] = { pv[3], pv[4], pv[5], pv[6] };
                    if (pq[3] > 0.0f) {
                        pq[0] = -pq[0]; pq[1] = -pq[1];
                        pq[2] = -pq[2]; pq[3] = -pq[3];
                    }
                    q_normalize(pq);
                    float pt[3] = { pv[0], pv[1], pv[2] };

                    float rot_t[3];
                    q_rotate(pq, abs_t, rot_t);
                    for (int i = 0; i < 3; ++i) abs_t[i] = rot_t[i] + pt[i];
                    float new_q[4];
                    q_mul(pq, abs_q, new_q);
                    memcpy(abs_q, new_q, sizeof(new_q));
                    parent = iqm.joints[parent].parent;
                }

                // CG_SkeletalPoseGetAttachment(): inverse of the absolute rotation.
                float attach_q[4];
                q_conj(abs_q, attach_q);
                q_normalize(attach_q);
                float attach_matrix[9];
                q_to_matrix3(attach_q, attach_matrix);

                double max_err = 0.0;
                for (int i = 0; i < 9; ++i)
                    max_err = std::max(max_err, (double)std::fabs(attach_matrix[i] - frames[f].axis[i]));
                check(max_err < 2e-4, "Warfork attachment orientation equals the MD3 tag basis");

                for (int i = 0; i < 3; ++i)
                    check_close(abs_t[i], frames[f].origin[i], 2e-3f, "Warfork attachment position equals the MD3 tag origin");
            }
        }
    }

    // --- static (single frame) MD3 ------------------------------------------
    // Warfork refuses to load an IQM without frames or animations
    // (ref_gl/r_skm.c: "has no animations"), so a static tag-only MD3 must
    // still export a one-frame animation.
    printf("== static single-frame MD3 ==\n");
    {
        std::vector<TagSample> static_frames(1);
        static_frames[0].origin[0] = -4.5f;
        static_frames[0].origin[1] = 0.0f;
        static_frames[0].origin[2] = -11.40625f;
        make_basis(0.0f, static_frames[0].axis);

        const std::string static_md3 = tmpdir + "test_md3_tags_static.md3";
        const std::string static_iqm = tmpdir + "test_md3_tags_static.iqm";
        if (!write_synthetic_md3(static_md3, static_frames)) {
            printf("FAILED to write static MD3\n");
            return 1;
        }

        gs_model* sm = gsk_load_md3_tags(static_md3.c_str());
        check(sm != nullptr, "static MD3 imports");
        if (sm) {
            check(sm->num_animations == 1, "static MD3 still produces one animation");
            gsk_convert_orientation(sm, GS_Y_UP_RIGHTHANDED, GS_WINDING_CCW);
            check(gsk_write_iqm(static_iqm.c_str(), sm, false), "static MD3 exports to IQM");
            gsk_free_model(sm);

            IqmFile siqm;
            check(read_iqm(static_iqm, siqm), "static IQM file parsed");
            check(siqm.num_frames >= 1, "static IQM has at least one frame");
            check(siqm.num_anims >= 1, "static IQM has at least one animation");
            check(!siqm.poses.empty(), "static IQM has poses");
        }
    }

    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
}
