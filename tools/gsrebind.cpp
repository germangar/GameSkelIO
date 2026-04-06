#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gameskelio.h"

// --- Portable Extraction Logic ---

/**
 * Helper to find an animation index by index string or name.
 */
int gsk_find_animation_index(const gs_model* model, const char* anim_target) {
    if (!model || !anim_target) return -1;
    char* endptr;
    long val = strtol(anim_target, &endptr, 10);
    if (*endptr == '\0') {
        return (int)val;
    } else {
        for (uint32_t i = 0; i < model->num_animations; ++i) {
            if (model->animations[i].name && strcmp(model->animations[i].name, anim_target) == 0) {
                return (int)i;
            }
        }
    }
    return -1;
}

/**
 * REBIND MODEL LOGIC
 * Extracts the core logic of the gsrebind tool into a single portable function.
 * 
 * 1. Captures the current bind pose as a new animation named "original_bind".
 * 2. Re-bases the entire model and all animations to match the target 'pose_anim_idx'.
 * 3. Reorders the animations to make the new bind pose the first animation (index 0).
 */
bool gsk_rebind_model(gs_model* model, int pose_anim_idx) {
    if (!model || pose_anim_idx < 0 || pose_anim_idx >= (int)model->num_animations) {
        return false;
    }

    // 1. Capture "Old Bind" as a new animation locally
    printf("Capturing current bind pose as 'original_bind'...\n");
    uint32_t old_anim_count = model->num_animations;
    model->animations = (gs_animation*)realloc(model->animations, (old_anim_count + 1) * sizeof(gs_animation));
    gs_animation* new_anim = &model->animations[old_anim_count];
    new_anim->name = strdup("original_bind");
    new_anim->num_bones = model->num_joints;
    new_anim->bones = (gs_bone_anim*)calloc(model->num_joints, sizeof(gs_bone_anim));
    
    for (uint32_t i = 0; i < model->num_joints; ++i) {
        gs_bone_anim* ba = &new_anim->bones[i];
        
        // Single frame at time 0.0
        ba->translation.num_keys = 1;
        ba->translation.times = (double*)malloc(sizeof(double));
        ba->translation.times[0] = 0.0;
        ba->translation.values = (float*)malloc(3 * sizeof(float));
        memcpy(ba->translation.values, model->joints[i].translate, 12);

        ba->rotation.num_keys = 1;
        ba->rotation.times = (double*)malloc(sizeof(double));
        ba->rotation.times[0] = 0.0;
        ba->rotation.values = (float*)malloc(4 * sizeof(float));
        memcpy(ba->rotation.values, model->joints[i].rotate, 16);

        ba->scale.num_keys = 1;
        ba->scale.times = (double*)malloc(sizeof(double));
        ba->scale.times[0] = 0.0;
        ba->scale.values = (float*)malloc(3 * sizeof(float));
        memcpy(ba->scale.values, model->joints[i].scale, 12);
    }
    model->num_animations++;

    // 2. Perform the Rebase
    printf("Rebinding to animation %d (%s)...\n", pose_anim_idx, model->animations[pose_anim_idx].name);
    if (!gsk_rebase_pose(model, pose_anim_idx)) {
        return false;
    }

    // 3. Reorder: move the new bind pose (pose_anim_idx) to index 0
    printf("Promoting T-Pose to animation 0...\n");
    gsk_move_animation(model, (uint32_t)pose_anim_idx, 0);

    return true;
}

// --- CLI Frontend ---

void print_usage(const char* prog) {
    printf("Usage: %s <input_file> <anim_idx_or_name> <output_file>\n", prog);
    printf("Example: %s player.iqm T-Pose player_tpose.iqm\n", prog);
    printf("Example: %s player.iqm 5 player_tpose.iqm\n", prog);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        print_usage(argv[0]);
        return 1;
    }

    const char* input_path = argv[1];
    const char* anim_target = argv[2];
    const char* output_path = argv[3];

    printf("GameSkelIO Rebind Tool (API v%d)\n", GAMESKELIO_VERSION);
    printf("Build: %s %s\n\n", __DATE__, __TIME__);

    // Load
    printf("Loading: %s...\n", input_path);
    gs_model* model = NULL;
    if (strstr(input_path, ".iqm")) model = gsk_load_iqm(input_path);
    else if (strstr(input_path, ".glb")) model = gsk_load_glb(input_path);
    else if (strstr(input_path, ".fbx")) model = gsk_load_fbx(input_path);

    if (!model) {
        fprintf(stderr, "Error: Could not load model %s\n", input_path);
        return 1;
    }

    // Resolve target animation
    int anim_idx = gsk_find_animation_index(model, anim_target);
    if (anim_idx < 0 || anim_idx >= (int)model->num_animations) {
        fprintf(stderr, "Error: Could not find animation '%s' (Index out of range or name not found)\n", anim_target);
        gsk_free_model(model);
        return 1;
    }

    // Rebind
    if (!gsk_rebind_model(model, anim_idx)) {
        fprintf(stderr, "Error: Rebind operation failed.\n");
        gsk_free_model(model);
        return 1;
    }

    // Save
    printf("Saving: %s...\n", output_path);
    bool success = false;
    if (strstr(output_path, ".iqm")) success = gsk_write_iqm(output_path, model, false);
    else if (strstr(output_path, ".glb")) success = gsk_write_glb(output_path, model);

    if (!success) {
        fprintf(stderr, "Error: Could not write model %s\n", output_path);
        gsk_free_model(model);
        return 1;
    }

    printf("Done!\n");
    gsk_free_model(model);
    return 0;
}
