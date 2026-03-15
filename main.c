#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include "gameskelio.h"

static bool file_exists(const char* path) {
    FILE* f = fopen(path, "rb");
    if (f) {
        fclose(f);
        return true;
    }
    return false;
}

static char* get_unique_path(const char* path) {
    if (!file_exists(path)) {
        return strdup(path);
    }

    const char* last_dot = strrchr(path, '.');
    size_t base_len;
    if (last_dot) {
        base_len = last_dot - path;
    } else {
        base_len = strlen(path);
    }

    char* base = (char*)malloc(base_len + 1);
    memcpy(base, path, base_len);
    base[base_len] = '\0';

    const char* ext = last_dot ? last_dot : "";

    int counter = 1;
    char* new_path = (char*)malloc(strlen(path) + 32);
    while (true) {
        sprintf(new_path, "%s_%d%s", base, counter, ext);
        if (!file_exists(new_path)) {
            free(base);
            return new_path;
        }
        counter++;
    }
}

static bool ends_with(const char* str, const char* suffix) {
    size_t len = strlen(str);
    size_t slen = strlen(suffix);
    if (len < slen) return false;
    for (size_t i = 0; i < slen; ++i) {
        if (tolower((unsigned char)str[len - slen + i]) != tolower((unsigned char)suffix[i]))
            return false;
    }
    return true;
}

static void* read_file_to_buffer(const char* path, size_t* out_size) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    *out_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    void* buf = malloc(*out_size);
    fread(buf, 1, *out_size, f);
    fclose(f);
    return buf;
}

static gs_legacy_framegroup* parse_cfg(const char* path, uint32_t* out_count) {
    FILE* f = fopen(path, "r");
    if (!f) return NULL;

    gs_legacy_framegroup* anims = NULL;
    *out_count = 0;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char* p = line;
        while (isspace(*p)) p++;
        if (!*p || !isdigit(*p)) continue;

        int first, last, loop;
        float fps;
        int fields = sscanf(p, "%d %d %d %f", &first, &last, &loop, &fps);
        if (fields < 2) continue;

        char* comment = strstr(p, "//");
        char* name = NULL;
        if (comment) {
            comment += 2;
            while (isspace(*comment)) comment++;
            char* end = comment;
            while (*end && !isspace(*end)) end++;
            size_t name_len = end - comment;
            if (name_len > 0) {
                name = malloc(name_len + 1);
                memcpy(name, comment, name_len);
                name[name_len] = '\0';
            }
        }

        anims = realloc(anims, (*out_count + 1) * sizeof(gs_legacy_framegroup));
        anims[*out_count].first_frame = first;
        anims[*out_count].num_frames = last - first + 1;
        anims[*out_count].fps = (fields >= 4) ? fps : 30.0f;
        anims[*out_count].name = name ? name : strdup("unnamed");
        (*out_count)++;
    }

    fclose(f);
    return anims;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <input.iqm/glb/skm> <output.glb/iqm/fbx> [--base] [--anim] [--qfusion]\n", argv[0]);
        return 1;
    }

    const char* in_path = argv[1];
    const char* out_path = argv[2];

    gs_model* model = NULL;
    bool qfusion = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--qfusion") == 0) {
            qfusion = true;
        }
    }

    // Load Phase
    if (ends_with(in_path, ".iqm")) {
        printf("Loading IQM (via buffer): %s...\n", in_path);
        size_t size;
        void* data = read_file_to_buffer(in_path, &size);
        if (data) {
            uint32_t num_anims = 0;
            gs_legacy_framegroup* overrides = NULL;
            
            // Priority 1: <model>.cfg
            char cfg_path[512];
            strncpy(cfg_path, in_path, sizeof(cfg_path));
            char* dot = strrchr(cfg_path, '.');
            if (dot) strcpy(dot, ".cfg");
            else strcat(cfg_path, ".cfg");

            if (file_exists(cfg_path)) {
                printf("Found animation config: %s\n", cfg_path);
                overrides = parse_cfg(cfg_path, &num_anims);
            } else {
                // Priority 2: animation.cfg in the same directory
                char* last_slash = strrchr(cfg_path, '/');
                char* last_back = strrchr(cfg_path, '\\');
                char* path_sep = (last_slash > last_back) ? last_slash : last_back;
                
                if (path_sep) {
                    strcpy(path_sep + 1, "animation.cfg");
                } else {
                    strcpy(cfg_path, "animation.cfg");
                }

                if (file_exists(cfg_path)) {
                    printf("Found animation config: %s\n", cfg_path);
                    overrides = parse_cfg(cfg_path, &num_anims);
                }
            }

            model = gsk_load_iqm_buffer(data, size, overrides, num_anims);
            
            free(data);
            if (overrides) {
                for (uint32_t i = 0; i < num_anims; ++i) free((void*)overrides[i].name);
                free(overrides);
            }
        }
    } else if (ends_with(in_path, ".skm") || ends_with(in_path, ".skp")) {
        printf("Loading SKM/SKP (via buffer): %s...\n", in_path);
        
        char base_path[512];
        strncpy(base_path, in_path, sizeof(base_path));
        char* dot = strrchr(base_path, '.');
        if (dot) *dot = '\0';

        char skm_path[512], skp_path[512];
        sprintf(skm_path, "%s.skm", base_path);
        sprintf(skp_path, "%s.skp", base_path);

        size_t skm_size, skp_size;
        void* skm_data = read_file_to_buffer(skm_path, &skm_size);
        void* skp_data = read_file_to_buffer(skp_path, &skp_size);

        if (skm_data && skp_data) {
            uint32_t num_anims = 0;
            gs_legacy_framegroup* overrides = NULL;
            char cfg_path[512];
            sprintf(cfg_path, "%s.cfg", base_path);

            if (file_exists(cfg_path)) {
                printf("Found animation config: %s\n", cfg_path);
                overrides = parse_cfg(cfg_path, &num_anims);
            } else {
                sprintf(cfg_path, "animation.cfg"); // Should ideally be in same dir
                if (file_exists(cfg_path)) {
                    printf("Found animation config: %s\n", cfg_path);
                    overrides = parse_cfg(cfg_path, &num_anims);
                }
            }

            model = gsk_load_skm_buffer(skm_data, skm_size, skp_data, skp_size, overrides, num_anims);

            free(skm_data);
            free(skp_data);
            if (overrides) {
                for (uint32_t i = 0; i < num_anims; ++i) free((void*)overrides[i].name);
                free(overrides);
            }
        }
    } else if (ends_with(in_path, ".glb") || ends_with(in_path, ".gltf")) {
        printf("Loading GLB (cgltf): %s...\n", in_path);
        model = gsk_load_glb(in_path);
    } else if (ends_with(in_path, ".fbx")) {
        printf("Loading FBX (ufbx): %s...\n", in_path);
        model = gsk_load_fbx(in_path);
    }

    if (!model) {
        fprintf(stderr, "Failed to load input file or unsupported format: %s\n", in_path);
        return 2;
    }

    model->qfusion = qfusion;

    // Universal Sanitization & Initialization
    if (!gsk_validate_skeleton(model)) {
        printf("Reordering skeleton for topological consistency...\n");
        gsk_reorder_skeleton(model);
    }
    gsk_compute_bind_pose(model);
    gsk_compute_bounds(model);

    size_t total_keys = 0;
    for (uint32_t i = 0; i < model->num_animations; ++i) {
        for (uint32_t j = 0; j < model->animations[i].num_bones; ++j) {
            total_keys += model->animations[i].bones[j].translation.num_keys + 
                          model->animations[i].bones[j].rotation.num_keys + 
                          model->animations[i].bones[j].scale.num_keys;
        }
    }

    printf("Model loaded: %u meshes, %u joints, %u animations (%zu keys)\n", 
           model->num_meshes, model->num_joints, model->num_animations, total_keys);
    if (model->has_bounds) {
        printf("Bounds: (%.3f %.3f %.3f) to (%.3f %.3f %.3f)\n", 
               model->mins[0], model->mins[1], model->mins[2],
               model->maxs[0], model->maxs[1], model->maxs[2]);
    }

    // Write Phase
    char* unique_out = get_unique_path(out_path);
    int ret_code = 0;

    if (ends_with(unique_out, ".iqm")) {
        printf("Writing IQM: %s...\n", unique_out);
        if (!gsk_write_iqm(unique_out, model)) ret_code = 3;
    } else if (ends_with(unique_out, ".fbx")) {
        bool write_base = true;
        bool write_anim = true;
        for (int i = 1; i < argc; ++i) {
            if (strcmp(argv[i], "--base") == 0) write_anim = false;
            if (strcmp(argv[i], "--anim") == 0) write_base = false;
        }

        if (write_base && !write_anim) printf("Writing FBX (Base Pose only): %s...\n", unique_out);
        else if (!write_base && write_anim) printf("Writing FBX (Animations only): %s...\n", unique_out);
        else printf("Writing FBX (Complete): %s...\n", unique_out);

        if (!gsk_write_fbx(unique_out, model, write_base, write_anim)) ret_code = 3;
    } else if (ends_with(unique_out, ".glb")) {
        printf("Writing GLB (cgltf): %s...\n", unique_out);
        if (!gsk_write_glb(unique_out, model)) ret_code = 3;
    } else {
        fprintf(stderr, "Unsupported output format: %s\n", unique_out);
        ret_code = 4;
    }

    gsk_free_model(model);
    free(unique_out);
    return ret_code;
}
