// MD3 tag-only loader.
//
// Quake 3's MD3 format stores attach points ("tags") as per-frame records of
// {name[64], origin[3], axis[3][3]}. Some assets in the wild are tags-only
// files (no surfaces at all) used purely as positioning/animation helpers,
// which conventional loaders reject. This loader reads ONLY the tags and
// produces a joint-only gs_model-compatible Model:
//
//   - A synthetic root joint (named after the file) is created at the origin;
//     every tag becomes one of its children. MD3 tags are flat (no hierarchy)
//     and live in model space, which we treat as the model/world origin.
//   - A tag's MD3 basis is stored as forward/right/up rows. Warfork's MD3
//     path exposes that basis directly, while its IQM client attachment path
//     returns the inverse of the absolute bone rotation. The imported tag
//     joint therefore stores the inverse MD3 basis (the transpose of a proper
//     rotation), so the final Warfork attachment reproduces the MD3 basis.
//   - The tag frames are always exported as a single sparse animation at
//     MD3_TAG_FPS with translation and rotation keys per tag (a static MD3
//     yields one held frame, because Warfork rejects anim-less IQM files).
//     Scale keys are not written.
//   - Meshes are never touched, so the result has 0 meshes / 0 vertices.
//
// Little-endian only (MD3 is always LE). Malformed-but-recoverable data is
// healed (clamped, renamed) rather than rejected wherever possible.

#include "md3_tags_loader.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <string>

// MD3 files carry no frame rate of their own. Sample tag frames at BASE_FPS
// (the same rate the IQM writer bakes animations at) so that converting a
// tag-only MD3 and exporting it to IQM preserves the source file's frame
// count exactly: an N-frame MD3 becomes duration (N-1)/BASE_FPS, which bakes
// back to N frames.
static const float MD3_TAG_FPS = BASE_FPS;

// Generous-but-bounded limits. Q3's own spec caps tags at 16, mod tools exceed it.
static const uint32_t MD3_TAGS_MAX_FRAMES = 4096;
static const uint32_t MD3_TAGS_MAX_TAGS = 256;
static const uint64_t MD3_TAGS_MAX_RECORDS = 65536ull; // frames * tags

// Header is 108 bytes: ident(4) version(4) filename(64) flags(4),
// 4 counts (4 each) and 4 offsets (4 each).
static const size_t MD3_HEADER_SIZE = 108;
static const size_t MD3_TAG_SIZE = 112; // name[64] + origin[12] + axis[36]

static const uint32_t MD3_IDENT_IDP3 = 0x33504449u; // "IDP3" little-endian

// Copy a fixed-size, possibly non-terminated name buffer into a clean string:
// stop at the first NUL, drop non-printables, trim trailing whitespace.
static std::string md3_clean_name(const char* raw, size_t max_len) {
    char buf[65];
    size_t n = std::min(max_len, (size_t)64);
    memcpy(buf, raw, n);
    size_t len = strnlen(buf, n); // names may be NUL-padded; never touch padding
    for (size_t i = 0; i < len; ++i) {
        unsigned char c = (unsigned char)buf[i];
        buf[i] = std::isprint(c) ? (char)c : '_';
    }
    buf[len] = '\0';
    while (len > 0 && std::isspace((unsigned char)buf[len - 1])) buf[--len] = '\0';
    return std::string(buf);
}

// Heal duplicate names (case-insensitive, consistent with the tooling's name
// guards) by appending _2, _3, ... Never fails.
static std::string md3_unique_name(std::set<std::string>& used, std::string name) {
    if (name.empty()) name = "tag";
    auto lower_of = [](const std::string& s) {
        std::string l = s;
        for (char& c : l) c = (char)std::tolower((unsigned char)c);
        return l;
    };
    std::string base = name;
    std::string key = lower_of(name);
    int suffix = 2;
    while (used.count(key)) {
        name = base + "_" + std::to_string(suffix++);
        key = lower_of(name);
        if (suffix > 1000) { name = base + "_" + std::to_string(rand()); key = lower_of(name); break; }
    }
    used.insert(key);
    return name;
}

// Filename without directory or extension, for the root joint / animation name.
static std::string md3_base_name(const char* path) {
    if (!path) return "";
    std::string s(path);
    size_t slash = s.find_last_of("/\\");
    if (slash != std::string::npos) s = s.substr(slash + 1);
    size_t dot = s.find_last_of('.');
    if (dot != std::string::npos) s = s.substr(0, dot);
    return s;
}

// Convert an MD3 tag basis to the local rotation stored by the IQM joint.
//
// MD3 stores three rows: forward, right and up. Warfork's alias loader copies
// those rows directly into its row-major mat3_t and converts that matrix to a
// quaternion. That quaternion is the orientation exposed by R_AliasModelLerpTag.
//
// The skeletal/IQM path is intentionally different:
// CG_SkeletalPoseGetAttachment() returns inverse(parent_rotation *
// joint_rotation). Our synthetic root is the identity, so the tag joint must
// contain the inverse of the MD3 tag rotation for the client to expose the
// original MD3 orientation. For a proper rotation that inverse is the
// transpose. Expressing the conversion explicitly avoids depending on TRS
// decomposition or on a matrix-layout convention.
//
// This is equivalent to Quat_FromMatrix3(MD3_basis), followed by a
// quaternion conjugate and normalization.
static void md3_tag_to_iqm_rotation(const float* axis /* [3][3] */, float q[4]) {
    // Warfork/QFusion Quat_FromMatrix3(), with the MD3 forward/right/up rows
    // copied directly into the engine's row-major matrix.
    const float tr = axis[0] + axis[4] + axis[8];
    if (tr > 0.00001f) {
        float s = sqrtf(tr + 1.0f);
        q[3] = s * 0.5f;
        s = 0.5f / s;
        q[0] = (axis[7] - axis[5]) * s;
        q[1] = (axis[2] - axis[6]) * s;
        q[2] = (axis[3] - axis[1]) * s;
    } else {
        int i = 0;
        if (axis[4] > axis[i * 3 + i]) i = 1;
        if (axis[8] > axis[i * 3 + i]) i = 2;
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;

        float s = sqrtf(axis[i * 3 + i] - (axis[j * 3 + j] + axis[k * 3 + k]) + 1.0f);
        q[i] = s * 0.5f;
        if (s != 0.0f) s = 0.5f / s;
        q[j] = (axis[j * 3 + i] + axis[i * 3 + j]) * s;
        q[k] = (axis[k * 3 + i] + axis[i * 3 + k]) * s;
        q[3] = (axis[k * 3 + j] - axis[j * 3 + k]) * s;
    }
    quat_normalize(q);

    // Inverse/conjugate of the MD3 alias rotation. Warfork's client performs
    // this inversion again when it reads the attachment from an IQM skeleton.
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
}

static uint32_t rd_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool load_md3_tags(const char* path, Model& out) {
    if (!path) return false;

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        fprintf(stderr, "MD3 tags: cannot open '%s'\n", path);
        return false;
    }
    std::streamsize fsize = file.tellg();
    file.seekg(0);
    std::vector<uint8_t> data((size_t)fsize);
    if (fsize < (std::streamsize)MD3_HEADER_SIZE || !file.read((char*)data.data(), fsize)) {
        fprintf(stderr, "MD3 tags: '%s' is too small to be an MD3 file\n", path);
        return false;
    }
    const uint8_t* buf = data.data();
    const uint64_t file_size = (uint64_t)data.size();

    // --- Header ---
    if (rd_u32(buf) != MD3_IDENT_IDP3) {
        fprintf(stderr, "MD3 tags: '%s' has invalid magic (expected IDP3)\n", path);
        return false;
    }
    // Version is expected to be 15, but heal: accept anything.
    // Header layout: ident(0) version(4) filename(8..71) flags(72)
    //                NUM_FRAMES(76) NUM_TAGS(80) NUM_SURFACES(84) NUM_SKINS(88)
    //                OFS_FRAMES(92) OFS_TAGS(96) OFS_SURFACES(100) OFS_EOF(104)
    uint32_t num_frames = rd_u32(buf + 76);
    uint32_t num_tags = rd_u32(buf + 80);
    uint32_t ofs_tags = rd_u32(buf + 96);

    if (num_tags == 0) {
        fprintf(stderr, "MD3 tags: '%s' contains no tags\n", path);
        return false;
    }
    if (num_tags > MD3_TAGS_MAX_TAGS || num_frames > MD3_TAGS_MAX_FRAMES ||
        (uint64_t)num_frames * num_tags > MD3_TAGS_MAX_RECORDS) {
        fprintf(stderr, "MD3 tags: '%s' exceeds limits (frames=%u tags=%u), clamping\n",
                path, num_frames, num_tags);
        if (num_tags > MD3_TAGS_MAX_TAGS) num_tags = MD3_TAGS_MAX_TAGS;
        if (num_frames > MD3_TAGS_MAX_FRAMES) num_frames = MD3_TAGS_MAX_FRAMES;
        while ((uint64_t)num_frames * num_tags > MD3_TAGS_MAX_RECORDS && num_frames > 1) num_frames /= 2;
    }

    // Clamp frames to what actually fits in the file (heal truncated files).
    uint64_t avail = (ofs_tags < file_size) ? (file_size - ofs_tags) : 0;
    uint64_t fits = avail / ((uint64_t)MD3_TAG_SIZE * num_tags);
    if (fits == 0) {
        fprintf(stderr, "MD3 tags: '%s' tag data is outside the file\n", path);
        return false;
    }
    uint32_t frames = (uint32_t)std::min<uint64_t>(num_frames, fits);

    // --- Joints: synthetic root + one child per tag ---
    std::string base = md3_base_name(path);
    if (base.empty()) base = "md3_root";
    for (char& c : base) if (!std::isprint((unsigned char)c)) c = '_';

    std::set<std::string> used_names;
    Joint root;
    root.name = md3_unique_name(used_names, base);
    root.parent = -1;
    root.translate[0] = root.translate[1] = root.translate[2] = 0.0f;
    root.rotate[0] = root.rotate[1] = root.rotate[2] = 0.0f; root.rotate[3] = 1.0f;
    root.scale[0] = root.scale[1] = root.scale[2] = 1.0f;
    out.joints.push_back(root);

    const uint8_t* tags = buf + ofs_tags;
    for (uint32_t t = 0; t < num_tags; ++t) {
        // Frame 0 of tag t defines the bind pose.
        const uint8_t* rec = tags + ((uint64_t)0 * num_tags + t) * MD3_TAG_SIZE;
        std::string name = md3_clean_name((const char*)rec, 64);
        if (name.empty()) name = "tag_" + std::to_string(t);

        Joint j;
        j.name = md3_unique_name(used_names, name);
        j.parent = 0;
        const float* origin = (const float*)(rec + 64);
        const float* axis = (const float*)(rec + 76);
        j.translate[0] = origin[0];
        j.translate[1] = origin[1];
        j.translate[2] = origin[2];
        md3_tag_to_iqm_rotation(axis, j.rotate);
        j.scale[0] = j.scale[1] = j.scale[2] = 1.0f;
        out.joints.push_back(j);
    }

    // --- Animation: one sparse clip from the tag frames at MD3_TAG_FPS ---
    //
    // A clip is always emitted, even for a single-frame (static) MD3: Warfork's
    // IQM loader rejects a model without frames or animations ("has no
    // animations", ref_gl/r_skm.c), and a static tag file would otherwise
    // export with num_frames = 0 / num_anims = 0. The single key becomes one
    // held frame in the exported IQM.
    {
        AnimationDef ad;
        ad.name = base; // animation names are a separate namespace from joints
        ad.duration = (double)(frames - 1) / MD3_TAG_FPS;
        ad.bones.resize(out.joints.size());

        for (uint32_t t = 0; t < num_tags; ++t) {
            BoneAnim& ba = ad.bones[t + 1]; // +1: skip the synthetic root
            ba.translation.times.resize(frames);
            ba.translation.values.resize((size_t)frames * 3);
            ba.rotation.times.resize(frames);
            ba.rotation.values.resize((size_t)frames * 4);

            for (uint32_t f = 0; f < frames; ++f) {
                const uint8_t* rec = tags + ((uint64_t)f * num_tags + t) * MD3_TAG_SIZE;
                const float* origin = (const float*)(rec + 64);
                const float* axis = (const float*)(rec + 76);

                double time = (double)f / MD3_TAG_FPS;
                ba.translation.times[f] = time;
                ba.translation.values[f * 3 + 0] = origin[0];
                ba.translation.values[f * 3 + 1] = origin[1];
                ba.translation.values[f * 3 + 2] = origin[2];

                float q[4];
                md3_tag_to_iqm_rotation(axis, q);
                ba.rotation.times[f] = time;
                ba.rotation.values[f * 4 + 0] = q[0];
                ba.rotation.values[f * 4 + 1] = q[1];
                ba.rotation.values[f * 4 + 2] = q[2];
                ba.rotation.values[f * 4 + 3] = q[3];
            }
        }
        out.animations.push_back(ad);
    }

    // Quake-family convention (same as IQM/SKM): Z-up, +X forward.
    // gskelview normalizes on load via gsk_convert_orientation.
    out.orientation = GS_Z_UP_RIGHTHANDED_X_FWD;
    out.winding = GS_WINDING_CW;

    printf("MD3 tags: '%s': %u tags, %u frames%s\n", path, num_tags, frames,
           frames > 1 ? " (animated)" : " (static)");
    return true;
}
