#pragma once
#include "model.h"

// Load an SKM/SKP model pair into the intermediate Model representation.
bool load_skm(const char* path, Model& out);

// Load an SKM model from memory. Requires both the mesh (.skm) and pose (.skp) buffers.
bool load_skm_from_memory(const void* skm_data, size_t skm_size, const void* skp_data, size_t skp_size, Model& out);
