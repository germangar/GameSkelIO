#pragma once
#include <vector>
#include <cstdint>
#include "model.h"

// Write the intermediate Model representation to an IQM file.
bool write_iqm(const Model& model, const char* output_path);

// Bake the intermediate Model representation into an IQM binary memory block.
std::vector<uint8_t> write_iqm_to_memory(const Model& model);
