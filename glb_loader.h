#pragma once
#include "model.h"

// Load a GLB or glTF file into the intermediate Model representation.
bool load_glb(const char* path, Model& out);

// Load a GLB or glTF model from a memory block.
bool load_glb_from_memory(const void* data, size_t size, Model& out);
