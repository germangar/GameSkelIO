#pragma once
#include <string>
#include <vector>
#include "model.h"

// Load an IQM file into the intermediate Model representation.
bool load_iqm(const char* path, Model& out);

// Load an IQM model from a memory block.
bool load_iqm_from_memory(const void* data, size_t size, Model& out);
