#include "anim_cfg.h"
#include <fstream>
#include <iostream>
#include <cstdio>   // sscanf
#include <cctype>   // isdigit

#ifndef BASE_FPS
#define BASE_FPS 30.0f
#endif

std::string find_animation_cfg(const std::string& iqm_path) {
    size_t last_dot   = iqm_path.find_last_of('.');
    size_t last_slash = iqm_path.find_last_of("/\\");

    std::string dir = (last_slash == std::string::npos)
                    ? ""
                    : iqm_path.substr(0, last_slash + 1);

    // Priority 1: <modelname>.cfg (e.g. tris.cfg)
    std::string model_cfg = (last_dot != std::string::npos)
                          ? iqm_path.substr(0, last_dot) + ".cfg"
                          : iqm_path + ".cfg";

    // Priority 2: animation.cfg in the same directory
    std::string generic_cfg = dir + "animation.cfg";

    if (std::ifstream(model_cfg).good())   return model_cfg;
    if (std::ifstream(generic_cfg).good()) return generic_cfg;
    return "";
}

std::vector<AnimConfigEntry> parse_animation_cfg(const std::string& path) {
    std::vector<AnimConfigEntry> entries;

    std::ifstream f(path);
    if (!f.is_open()) return entries;

    std::string line;
    while (std::getline(f, line)) {
        // Trim leading whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        if (line.empty()) continue;

        // Skip Warsow-specific non-animation commands
        if (line.find("sex")          == 0 ||
            line.find("rootanim")     == 0 ||
            line.find("tagmask")      == 0 ||
            line.find("rotationbone") == 0) continue;

        // Animation lines start with a digit
        if (!std::isdigit((unsigned char)line[0])) continue;

        int   first = 0, last = 0, loop = 0;
        float fps = BASE_FPS;

        // Require at least two integers (first, last)
        int fields = sscanf(line.c_str(), "%d %d %d %f", &first, &last, &loop, &fps);
        if (fields < 2) continue;

        AnimConfigEntry ace;
        ace.first_frame = first;
        ace.last_frame  = last;
        ace.loop_frames = (fields >= 3) ? loop : 0;
        ace.fps         = (fields >= 4) ? fps : BASE_FPS;

        // Extract name from trailing // comment
        size_t comment_pos = line.find("//");
        if (comment_pos != std::string::npos) {
            std::string comment = line.substr(comment_pos + 2);
            // Trim leading and trailing whitespace
            size_t start = comment.find_first_not_of(" \t\r\n");
            if (start != std::string::npos) {
                size_t end = comment.find_last_not_of(" \t\r\n");
                ace.name = comment.substr(start, end - start + 1);
            } else {
                ace.name = "unnamed_" + std::to_string(entries.size());
            }
        } else {
            ace.name = "unnamed_" + std::to_string(entries.size());
        }

        entries.push_back(ace);
    }

    // Ensure we have at least one "base" animation if file was empty
    if (entries.empty()) {
        entries.push_back({"base", 0, 0, 0, BASE_FPS});
    }

    return entries;
}
