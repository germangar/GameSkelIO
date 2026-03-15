#pragma once
#include "model.h"

// Load a Binary or ASCII FBX file using the ufbx library.
bool load_fbx(const char* path, Model& out);

// Load an FBX model from a memory block.
bool load_fbx_from_memory(const void* data, size_t size, Model& out);
