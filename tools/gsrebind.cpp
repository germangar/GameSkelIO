#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gameskelio.h"

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
    printf("Build: %s %s [ANTIGRAVITY_FIX_V3]\n\n", __DATE__, __TIME__);
    printf("Loading: %s...\n", input_path);

    gs_model* model = NULL;
    if (strstr(input_path, ".iqm")) model = gsk_load_iqm(input_path);
    else if (strstr(input_path, ".glb")) model = gsk_load_glb(input_path);
    else if (strstr(input_path, ".fbx")) model = gsk_load_fbx(input_path);

    if (!model) {
        fprintf(stderr, "Error: Could not load model %s\n", input_path);
        return 1;
    }

    int anim_idx = -1;
    char* endptr;
    long val = strtol(anim_target, &endptr, 10);
    if (*endptr == '\0') {
        anim_idx = (int)val;
    } else {
        for (uint32_t i = 0; i < model->num_animations; ++i) {
            if (model->animations[i].name && strcmp(model->animations[i].name, anim_target) == 0) {
                anim_idx = (int)i;
                break;
            }
        }
    }

    if (anim_idx < 0 || anim_idx >= (int)model->num_animations) {
        fprintf(stderr, "Error: Could not find animation '%s' (Index out of range or name not found)\n", anim_target);
        gsk_free_model(model);
        return 1;
    }

    // 1. Capture "Old Bind" as a new animation locally (per strategy)
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
    printf("Rebinding to animation %d (%s)...\n", anim_idx, model->animations[anim_idx].name);
    if (!gsk_rebase_pose(model, anim_idx)) {
        fprintf(stderr, "Error: Rebase failed (is the target animation static?)\n");
        gsk_free_model(model);
        return 1;
    }

    // 3. Reorder: move the new bind pose (anim_idx) to index 0
    printf("Promoting T-Pose to animation 0...\n");
    gsk_move_animation(model, (uint32_t)anim_idx, 0);

    // 4. Save
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

/*REBIND TOOL FINAL SOLUTION
================================

1. THE INITIAL PROBLEM
---------------------
The rebinding tool was failing to produce perfect animations after changing the character's rest pose (e.g., from T-pose to A-pose). 
Symptoms included:
- Stretching/Orbiting: Bones swinging in massive arcs instead of pivoting at joints.
- Axis Scrambling: Left arm twisting 90 or 180 degrees during movement.
- Frozen Limbs: Un-animated joints (like the right arm) ignoring the new bind pose or snapping back to the old one.

2. FAILED ATTEMPTS & ANALYSIS
-----------------------------
- Attempt 1 (World-Space Invariance): Tried to warp world-space matrices. This caused "orbiting" because vertices rotated around old joint centers instead of new ones.
- Attempt 2 (Formulaic Local Retargeting): Tried to apply deltas using L_new = L_new_bind * (L_old_bind^-1 * L_orig). This mathematically scrambled rotation axes, causing the 180-degree "spinning" bug.

3. THE FINAL SOLUTION: MATHEMATICAL CANCELLATION
------------------------------------------------
The breakthrough was realizing that if we change BOTH the mesh vertices AND the Inverse Bind Matrices (IBMs) to the new pose, they perfectly cancel each other out in the skinning equation.

Mathematical Proof:
Original Skinning: V_anim = W_orig_anim * IBM_orig * V_orig
New Skinning:      V_anim = W_new_anim * IBM_new * V_warped

Substituting V_warped (which we physically moved to the new pose):
V_warped = W_new_bind * IBM_orig * V_orig

Therefore:
V_anim = W_new_anim * IBM_new * (W_new_bind * IBM_orig) * V_orig
Since (IBM_new * W_new_bind) = Identity:
V_anim = W_new_anim * IBM_orig * V_orig

To make this equal the original V_anim, we simply need:
W_new_anim == W_orig_anim

Conclusion: Since the hierarchy topology is identical, if we keep the LOCAL TRS keys exactly the same as the original file, the world-space geometry will be 100% identical to the original game.

4. THE "UN-ANIMATED JOINT" FIX
------------------------------
The reason the right arm looked broken was because it had NO keys. Game engines fall back to the CURRENT bind pose for empty tracks. Since the bind pose changed to A-pose, but the other arm was still in T-pose (via its old keys), the character looked deformed.

Final Logic Implemented:
1. Mesh Warping: Physically move vertices to match the new A-pose.
2. IBM Recalculation: Update IBMs to match the new joint positions.
3. Unified Timeline: Collect all timestamps from every bone.
4. Dense Baking: For EVERY joint in the skeleton, explicitly evaluate its original local TRS (even if un-animated) and write a new keyframe into the track.
   - If a bone was animated, it keeps its original motion.
   - If a bone was NOT animated, it now has an explicit key forcing it to stay in its original T-pose position during that animation.

5. RESULT
---------
The model now STANDS in the new A-pose by default, but MOVES in the original T-pose world-space coordinates during animation. This allows different models to share the same animations flawlessly as long as they are rebound to the same shared A-pose standard.
*/