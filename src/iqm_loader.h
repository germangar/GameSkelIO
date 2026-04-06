#pragma once
#include <string>
#include <vector>
#include "model.h"
#include "gameskelio.h"

// Load an IQM file into the intermediate Model representation.
bool load_iqm(const char* path, Model& out);

// Load an IQM model from a memory block.
// Optional: provide external framegroup overrides (e.g. from animation.cfg).
bool load_iqm_from_memory(const void* data, size_t size, Model& out, const gs_legacy_framegroup* external_anims = nullptr, uint32_t num_external_anims = 0);
