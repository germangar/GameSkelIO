#pragma once
#include <vector>
#include <cstdint>
#include "model.h"
#include "gameskelio.h"

// Write the intermediate Model representation to an IQM file.
bool write_iqm(const Model& model, const char* output_path, bool force_single_anim = false);

// Bake the intermediate Model representation into an IQM binary memory block.
// Optional: returns calculated frame metadata in 'out_metadata'.
std::vector<uint8_t> write_iqm_to_memory(const Model& model, bool force_single_anim = false, std::vector<gs_legacy_framegroup>* out_metadata = nullptr);
