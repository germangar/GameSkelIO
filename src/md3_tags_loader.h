#pragma once
#include "model.h"

// Load ONLY the tags (attach points) of an MD3 file into a joint-only Model.
// Meshes and skins are ignored entirely; tag frames become a single sparse
// animation sampled at BASE_FPS (so baked IQM frame counts match the source
// MD3). The animation is emitted even for single-frame MD3s, because Warfork
// refuses to load an IQM without frames/animations. The MD3 model space origin
// is treated as the model/world origin: a synthetic root joint is created and
// every tag becomes one of its children, with per-frame origin/orientation keys.
bool load_md3_tags(const char* path, Model& out);
