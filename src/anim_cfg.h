#pragma once
#include <string>
#include <vector>

struct AnimConfigEntry {
    std::string name;
    int first_frame;
    int last_frame;
    int loop_frames;
    float fps;
};

// Locate an animation config alongside the given IQM path.
// Checks <model>.cfg first, then animation.cfg in the same directory.
// Returns empty string if not found.
std::string find_animation_cfg(const std::string& iqm_path);

// Parse a Warsow/Warfork-style animation.cfg.
// Returns a list of animation definitions with their raw frame ranges.
// Legacy files list numeric-first entries and replace the model's animation
// list entirely. Files containing an "animations" keyword switch to the
// name-first format; their entries extend or override the animations embedded
// in the model instead. If merge_with_builtin is non-null it reports which
// behaviour applies.
std::vector<AnimConfigEntry> parse_animation_cfg(const std::string& path, bool* merge_with_builtin = nullptr);
