# GameSkelIO — Agent Instructions

C/C++ library that transcodes skeletal models and animations between FBX, GLB/glTF, IQM and
SKM/SKP in memory or on disk, with CLI tools (`gskelconv`, `gsrebind`).

Read `CONTEXT.md` before changing anything; it is the source of truth for architecture, the
`gs_model` layout and the writer constraints. Re-read **§8 (FBX Writer: Maya Compatibility
Constraints)** before touching `src/fbx_writer.cpp`, `src/libs/fbx.cpp`, or anything that changes
`gsk_write_fbx()` output.

## Hard rules

- **The §8 FBX constraints are invariants, not style preferences.** Maya's FBX SDK rejects
  structurally imperfect files *silently* (empty scene, no error). Blender is forgiving, so a
  passing Blender import does not prove the file is valid. Verify with headless `mayapy` node
  counting (snippet in §8) and, for the skinned/oriented result, compare Maya world bounding boxes
  against the GLB export of the same model.
- **This is a pure skeletal-model library.** Do not add game- or viewer-specific importers here
  (the MD3 tag importer lives in `gskelview/model_md3tags.cpp`).
- **Committed binaries are part of the deliverable.** After any source change rebuild with
  `make -j8` and keep these in sync at the repo root: `libgameskelio.a`, `libgameskelio.dll.a`,
  `libgameskelio_x64.dll`, `gskelconv.exe`, `gsrebind.exe`. Downstream projects (`gskelview`) vendor
  `libgameskelio.a` / `libgameskelio.dll.a`, so stale binaries silently undo your fix.
- Keep the public API in `gameskelio.h` and the DLL/static builds in lockstep (API version in sync).
- **Commits:** the user normally commits this repo themselves. Only commit when explicitly asked.
- Local verification assets: `tris.iqm` (34 joints, 1 mesh, 50 animation clips) and the CLI tools
  `gskelconv.exe` / `gsrebind.exe`. Test models ignored by git (`*.iqm`, `*.glb`, `*.fbx`) may still
  exist locally — use them, don't delete them.

## Build

```bash
make clean
make -j8     # libgameskelio.a, libgameskelio_x64.dll, gskelconv.exe, gsrebind.exe
```

Linking requires a C++ linker (e.g. `g++`) to resolve internal dependencies.

## Typical conversion check

```bash
./gskelconv.exe tris.iqm out.fbx        # then import out.fbx in Maya and Blender
./gskelconv.exe model.iqm model.glb     # reference for scale/orientation comparisons
```
