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
std::vector<AnimConfigEntry> parse_animation_cfg(const std::string& path);
