#pragma once
#include "model.h"
#include "gameskelio.h"

// Load an SKM/SKP model pair into the intermediate Model representation.
bool load_skm(const char* path, Model& out);

// Load an SKM model from memory. Requires both the mesh (.skm) and pose (.skp) buffers.
// Optional: provide external framegroup overrides (e.g. from animation.cfg).
bool load_skm_from_memory(const void* skm_data, size_t skm_size, const void* skp_data, size_t skp_size, Model& out, const gs_legacy_framegroup* external_anims = nullptr, uint32_t num_external_anims = 0);
