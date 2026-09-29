#include "anim_cfg.h"
#include <fstream>
#include <iostream>
#include <cstdio>   // sscanf
#include <cctype>   // isdigit
#include <sstream>  // istringstream

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

    // "animations" keyword starts the name-first block (new format).
    // Absent, we treat lines as the old numeric-first format.
    bool name_first_block = false;

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

        if (line.find("animations") == 0) {
            name_first_block = true;
            continue;
        }

        AnimConfigEntry ace;
        ace.loop_frames = 0;
        ace.fps = BASE_FPS;

        if (name_first_block) {
            // New format: <name> <first> <last> <loop> <fps>
            std::istringstream ss(line);
            if (!(ss >> ace.name)) continue;
            if (!(ss >> ace.first_frame)) continue;
            if (!(ss >> ace.last_frame)) continue;
            ss >> ace.loop_frames;
            ss >> ace.fps;
        } else {
            // Old format: <first> <last> <loop> <fps> // <name>
            if (!std::isdigit((unsigned char)line[0])) continue;

            int   first = 0, last = 0, loop = 0;
            float fps = BASE_FPS;

            // Require at least two integers (first, last)
            int fields = sscanf(line.c_str(), "%d %d %d %f", &first, &last, &loop, &fps);
            if (fields < 2) continue;

            ace.first_frame = first;
            ace.last_frame  = last;
            ace.loop_frames = (fields >= 3) ? loop : 0;
            ace.fps         = (fields >= 4) ? fps : BASE_FPS;

            // Extract name: prefer trailing // comment, otherwise try 5th field
            std::string name;
            size_t comment_pos = line.find("//");
            if (comment_pos != std::string::npos) {
                name = line.substr(comment_pos + 2);
            } else {
                // Find where the numbers end to see if there's a name after them
                const char* p = line.c_str();
                int matched = 0;
                while (*p && matched < fields) {
                    while (*p && std::isspace((unsigned char)*p)) p++;
                    if (!*p) break;
                    // Move past the number
                    while (*p && !std::isspace((unsigned char)*p)) p++;
                    matched++;
                }
                while (*p && std::isspace((unsigned char)*p)) p++;
                if (*p) name = p;
            }

            // Clean and trim the name
            ace.name.clear();
            if (!name.empty()) {
                size_t start = name.find_first_not_of(" \t\r\n");
                if (start != std::string::npos) {
                    size_t end = name.find_last_not_of(" \t\r\n");
                    ace.name = name.substr(start, end - start + 1);
                }
            }

            // Fallback if still empty
            if (ace.name.empty()) {
                ace.name = "unnamed_" + std::to_string(entries.size());
            }
        }

        // Fallback if name is empty (new format only)
        if (ace.name.empty()) {
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
