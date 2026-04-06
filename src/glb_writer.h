#pragma once
#include <vector>
#include <cstdint>
#include "model.h"

// Write the intermediate Model representation to a GLB file.
bool write_glb(const Model& model, const char* output_path);

// Bake the intermediate Model representation into a GLB binary memory block.
std::vector<uint8_t> write_glb_to_memory(const Model& model);
