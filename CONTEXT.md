# AI Agent Project Guide: GameSkelIO

This document provides essential technical context for AI agents to understand the architecture, data flow, and operational constraints of the GameSkelIO project.

## 1. Project Mission
High-performance 3D skeletal model and animation transcoding. GameSkelIO serves as a high-fidelity bridge between modern formats (GLB, FBX) and engine-ready legacy formats (IQM, SKM).

## 2. Core Architecture: Self-Describing & Automated
The library avoids forcing data into a single internal "canonical" state, opting instead for a self-describing middleware approach.

- **Native-Format Loaders**: Formats are loaded in their **native orientation and winding order**. A GLB remains Y-Up/CCW, and an IQM remains Z-Up/CW.
- **Self-Describing `gs_model`**: The root container tracks its own `orientation` and `winding` state. This metadata allows processing tools to behave context-sensitively.
- **Target-Standard Writers**: Exporters automatically conform the model to their format's required standard (e.g., Y-Up for FBX, Z-Up for IQM) via transparent, in-place conversion on temporary copies.
- **Zero-Copy Orientation API**: `gsk_convert_orientation` performs math transformations directly within existing vertex and matrix buffers, avoiding costly reallocations.
- **Memory-First Design**: All core logic operates on memory buffers (`void* data, size_t size`). Path-based functions are mere wrappers that handle file I/O.

## 3. Data Hierarchy & Operational Constraints
### Data Blocks
- **Geometry**: Parallel vertex arrays (Positions, Normals, UV0/UV1, Tangents, Colors).
- **Skinning**: Bone influences (4-weight limit per vertex) and Joint Indices.
- **Skeleton**: Topological hierarchy with Local TRS and Inverse Bind Matrices (IBMs).
- **Animations**: Sparse, timestamp-based TRS channels.
- **Baked Motion**: On-demand generation of dense `T3, R4, S3` frame buffers.

### Constraints
- **Topological Order**: Joints in the skeleton must be sorted (Parent index < Child index) for hierarchical calculations. `reorder_skeleton()` handles this.
- **Welded Vertices**: Models are **deduplicated on import** ("welded"). Native loaders (FBX, GLB, IQM, SKM) weld automatically in `gsk_load_*`; loaders outside the library (e.g. Assimp-based) must call `gsk_weld_model()` explicitly. Welding merges vertices carrying bit-identical attributes (position, normal, tangent, color, UVs, skin indices/weights), remaps indices and fixes mesh ranges — it is lossless, idempotent and never removes triangles. Engines that extrude outlines along vertex normals need this shared vertex pool. Models with morph targets skip welding (per-vertex delta remapping is not implemented yet).
- **Winding Order**: 
    - **CCW**: Standard for GLB/FBX.
    - **CW**: Standard for IQM/SKM.
- **Math**: Column-Major `gs_mat4`. Rotation channels use `xyzw` quaternions.

## 4. Material & Texture System
GameSkelIO bridges the gap between PBR and Legacy (Phong/Lambert) workflows.

- **PBR Suffix Heuristic**: If format-specific PBR flags are missing, the library scans texture names for suffixes like `_albedo`, `_metallic`, `_roughness` to infer intent.
- **Automated PBR Upgrade**: When exporting legacy formats (IQM/SKM) to GLB, materials are automatically upgraded to Metallic-Roughness (Glossiness inverted to Roughness, Metallic set to 0.0).
- **Embedded Textures**: Raw image binary data (PNG/JPG) is preserved in `gs_texture_buffer` arrays and can be retrieved via the `gsk_get_embedded_texture` API.
- **Path Sanitization**: Directory structures and virtual prefixes (e.g., `embedded://`) are stripped during export to ensure clean, relative URIs.

## 5. Specialized Pipelines
- **FBX Logic**: Enforces a `UnitScaleFactor` of `100.0` (centimeter-relative, i.e. meters). Uses strict Object-Property (OP) connection logic to prevent connections-block corruption in binary exports. Maya's FBX SDK fails **silently** (empty scene, no error) on structural omissions that Blender tolerates — the full list of mandatory constraints is in **§8**. The writer also computes the bind pose itself if the incoming model lacks `world_matrices` (GLB loads don't provide them); without this the skin-cluster loop reads past the end of the array and crashes.
- **Vertex Welding**: `Model::weld()` (see §3) runs in the shared loader finalizer (`finalize_loaded` in `gameskelio.cpp`) **before** tangent computation, so computed tangents are smooth across shared vertices. `gsk_weld_model()` exposes the same operation for externally loaded models. Models already welded pay one cheap no-op pass.
- **IQM root-frame contract with consumers**: the IQM writer premultiplies its own orientation conversion onto parent==-1 root joints. Engines that apply the baked root rotation to the rendered model (validated: Warfork/QFusion) therefore expect the incoming model's root joints in their **authored** state — natively-Y-up sources (GLB/FBX) satisfy this naturally; loaders that canonicalize to Y-up before exporting (gskelview's MD3/IQM/SKM paths) must strip that canonicalization premultiply from the root skeleton at export time (implemented in gskelview `model_main_save_iqm`, 17/11/2026). Do not "fix" the writer itself.
- **GLB Logic**: Enforces 4-byte chunk alignment. Uses magic-byte sniffing for extensionless embedded images.
- **Animation Config (`animation.cfg`)**: Path-based IQM/SKM loads look for `<model>.cfg` first, then `animation.cfg` in the same directory. Without an `animations` keyword the file uses the legacy numeric-first layout (`<first> <last> <loop> <fps> // <name>`) and **replaces** the model's embedded animation list. With the keyword, entries use the name-first layout (`<name> <first> <last> <fps>`; the interim `<name> <first> <last> <loop> <fps>` is also accepted) and **merge** with the embedded animations: a case-insensitive name match replaces that entry in place, anything else is appended. Nameless embedded framegroups are ignored while merging. A config with no animation entries at all (commands only, e.g. `loopframes`/`tagmask`) leaves the embedded list untouched. `sex`, `rootanim`, `rotationbone`, `tagmask`, `loopframes` and `leaningScale` are skipped.
- **Rebinding (`gsk_rebase_pose`)**: Changes rest poses while preserving animation visual invariance by physically warping vertices and mathematically updating IBMs to cancel out the transformation during skinning.

## 6. Component Map
- **`gameskelio.h`**: Public C API. Source of truth for data structures.
- **`gameskelio.cpp`**: Core bridge logic and memory management.
- **`model.h`**: Internal C++ container logic and sampling helpers.
- **`orientation.cpp`**: Coordinate system transformation math.
- **`math_utils.h`**: Linear algebra primitives (Mat4, Quat, Vec3).
- **`glb_writer.cpp` / `fbx_writer.cpp`**: High-fidelity format-specific exporters.

## 7. Build & Maintenance
- **Makefile**: `make clean; make -j8` produces `libgameskelio.a`, `libgameskelio_x64.dll`, and CLI tools.
- **Versioning**: API versions are tracked in `gameskelio.h` (`GAMESKELIO_VERSION`, currently 5). Ensure DLLs and headers are updated in sync.
- **Golden Revision**: `c21f67b1` is the reference point for FBX writer stability. **Caution**: The FBX Object-Property (OP) hierarchy is extremely fragile; structural deviations in connection logic will likely result in **disconnected skinning links** between joints and meshes, causing animations to become inert (tracks are preserved but fail to drive model deformation).

## 8. FBX Writer: Maya Compatibility Constraints

Maya's FBX SDK (`fbxmaya`) is far stricter than Blender's importer, and it fails **silently** — the
import "succeeds" but produces an empty scene, with no exception, warning or log line. Every
constraint below was isolated empirically by bisecting a Blender-authored and a Maya-authored file
against the writer's output and re-importing headlessly. They are implemented in
`src/fbx_writer.cpp` / `src/libs/fbx.cpp` and must not be regressed.

**Fast feedback loop**: Maya's silence means node counting is the only practical test. Do not judge
an FBX change from a Blender import alone.

```python
# mayapy.exe check.py
import maya.standalone; maya.standalone.initialize('Python')
import maya.cmds as cmds
cmds.loadPlugin('fbxmaya')
cmds.file(new=True, force=True)
cmds.file(r"C:\path\model.fbx", type='FBX', i=True, options='v=0;', namespace='t')
tr = [t for t in (cmds.ls(transforms=True) or []) if t not in ('front','persp','side','top')]
print('joints=%d meshes=%d' % (
    sum(1 for t in tr if cmds.nodeType(t) == 'joint'),
    sum(1 for t in tr if cmds.listRelatives(t, shapes=True, noIntermediate=True, type='mesh'))))
```

Blender equivalent: `blender.exe --background --python check.py`, counting `bpy.data.objects` by
type and `bpy.data.actions`. Blender 5 note: an action only drives the **active** object (action
slots) — select the armature before switching clips, or assign
`arm.animation_data.action` explicitly, otherwise even a correct file looks unlinked.

### 8.1 Binary container (version 7400)

```
"Kaydara FBX Binary  "   21 bytes (two trailing spaces)
1A 00                    2 bytes
uint32 version           7400
record stream            uint32 end_offset, uint32 num_properties, uint32 property_list_len
                         uint8 name_len, name[], properties[], nested records
                         terminated by a 13-byte null record
footer block             176 bytes, see 8.2
```

Properties use type codes `Y I L F D C S R` plus arrays `i l f d b`. Object names encode
`Name\x00\x01Class` (FBX's `Name::Class` with `::` replaced by `0x00 0x01`), e.g.
`Bip01\x00\x01Model`. Polygon indices keep the FBX end-of-polygon convention `idx ^ -1` for the
last index of each triangle.

### 8.2 Footer block and the FileId triple

`src/libs/fbx.cpp` (~L831) writes this exact 176-byte block; a run of nulls is treated as a corrupt
file by the SDK (Blender tolerates it, which is why this went unnoticed for so long).

| Bytes | Content | Value |
| --- | --- | --- |
| 16 | creator/footer code | `fa bc ab 09 d0 c8 d4 66 b1 76 fb 83 1c f7 26 7e` |
| 20 | padding (reserved) | zero |
| 4 | file version | `7400` |
| 120 | padding/alignment | zero |
| 16 | magic | `f8 5a 8c 6a de f5 d9 7e ec e9 0c e3 75 8f 29 0b` |

Two **top-level** records right after `FBXHeaderExtension` must match that code:

```
FileId      : raw 16 bytes = 28 b3 2a eb b6 24 cc c2 bf c8 b0 2a a9 2b fc f1
CreationTime: "1970-01-01 10:00:00:000"
```

The SDK derives the footer creator code from this pair (falling back to the well-known code above);
Blender hardcodes the same constants in `io_scene_fbx/encode_bin.py`, which is why Blender-authored
files are accepted. **Evidence**: flipping a single GUID byte of an untouched, Maya-importable
Blender file makes Maya import nothing. Keep the three values as one constant, matched triple.
(`CreationTimeStamp` inside `FBXHeaderExtension` holds the real wall clock and is *not* validated.)

### 8.3 GlobalSettings: axes and units

The writer converts to Y-up/CCW (`convert_orientation(model, GS_Y_UP_RIGHTHANDED, GS_WINDING_CCW)`)
and then declares that space: `UpAxis=1`, `UpAxisSign=1`, `FrontAxis=2`, `FrontAxisSign=1`,
`CoordAxis=0`, `CoordAxisSign=1`, `OriginalUpAxis=1`, `OriginalUpAxisSign=1`.

`UnitScaleFactor = 100.0` means **meters**, because the factor is relative to FBX's internal
centimeters. GLB/OBJ are unitless and Blender treats 1 unit = 1 m, so 100 is what makes an FBX land
at the same physical size as the GLB/OBJ of the same model (launcher: 38.47 long axis in both).
`target_unit_meters = 0.0f` on the loader side keeps round trips raw.

**Do not "fix" the 100× display in Maya**: with meters declared, a 38-unit weapon shows ~3800 in the
channel box because the scene unit is cm. That is correct, not a scale bug.

### 8.4 `Model` records need two properties

Both joint Models and mesh Models must carry, in `Properties70`:

```
DefaultAttributeIndex  int   = 0
InheritType            enum  = 1
```

Without them the SDK parses the file but **drops the models**: joints arrive as plain transforms (no
skeleton, no skin binding) and skinned meshes never appear. Joints additionally need
`RotationOrder=0` (XYZ), local TRS, `Version=232`, and a paired `NodeAttribute` of class `LimbNode`
with `TypeFlags="Skeleton"`; mesh Models use `Version=232`, class `"Mesh"`. Euler angles are derived
via `stabilize_trs()` on a copy so near-identity rotations stay clean.

### 8.5 Every `LayerElement*` needs a `Name` child

All seven emitted kinds (`LayerElementNormal`, `LayerElementUV` for sets 0 and 1,
`LayerElementTangent`, `LayerElementBinormal`, `LayerElementColor`, `LayerElementMaterial`) include
a `Name` record (empty string is fine). Blender's importer asserts without it; the SDK tolerates it,
so only a Blender test catches the regression.

### 8.6 Skinning: cluster `Transform` must be the inverse bind matrix

FBX evaluates skinning as `v_skinned = TransformLink * Transform * v_local`, where `TransformLink`
is the joint's global bind matrix. At bind pose the product must be **identity** for the mesh to
stay where its vertices put it. Writing `Transform = identity` therefore multiplies every vertex by
the joint's world matrix — and the skeleton root carries the Z-up→Y-up conversion
(`Lcl Rotation = [-90, -90, 0]`), so already-converted vertices got converted a second time.

Symptom: in Maya the mesh is rotated away from its own skeleton/tags (launcher muzzle pointing up
while `tag_flash` sat sideways), while GLB and Blender both looked correct — because Blender's bind
matrices cancel it out.

Fix (~L660): `Transform = mat4_invert(in.world_matrices[i])`.

After the fix, Maya world measurements match the Blender-authored reference to rounding:

| | writer | reference |
| --- | --- | --- |
| launcher main mesh dims | 7.89 / 13.36 / 38.47 | 7.89 / 13.36 / 38.47 |
| launcher main mesh center | -0.01 / 4.40 / 11.58 | -0.01 / 4.40 / 11.58 |
| `tag_flash` world position | 0.00 / 390.62 / 2068.75 | 0.00 / 390.63 / 2068.75 |

(Maya cm; ÷100 = model units.)

Related trap: **mesh Models stay connected to the scene root (id 0)**, not to `attached_joint`.
Parenting them under the joint was tried and reverted — it changes world placement because the
vertices are already in converted space. Consistency comes solely from the bind delta above.

Connections that must stay exact: `Geometry → Mesh Model`, `Mesh Model → scene root`,
`Cluster → Skin Deformer`, `Skin → Geometry`, `Joint → parent joint | scene root`,
`NodeAttribute → Model`. A `Pose`/`BindPose` record lists every joint with its world matrix (same
source as the cluster `TransformLink`); Maya uses it as the rest pose.

### 8.7 Animation records

One `AnimationStack` + `AnimationLayer` per clip, `AnimationCurveNode` per joint property and
`AnimationCurve` per channel; `KeyTime` is in FBX ticks (`1 s = 46186158000`, `KTIME_ONE_SECOND`).
Verified on `tris.iqm` (50 clips): 50 stacks, 50 layers, 5100 curve nodes, 15300 curves, first clip
`LocalStart/LocalStop` = 0.0 → 0.75 s, and Blender evaluates real pose changes per clip.
Clip names appear as `<Armature>|<CLIP>` in Blender versus `RootNode|RootNode|<CLIP>` for a Blender
round-trip — cosmetic, not a linking fault. Single-pose clips (`BOTH_DEAD1`, `UI_POSE`) look static
by design.

### 8.8 Accepted deviations — do not "correct" these

Blender writes the same shapes and Maya accepts them; changing them risks regressions:

- `Deformer` record *names* (`Skin\x00\x01Deformer`, `Cluster_<joint>\x00\x01SubDeformer`) are cosmetic.
- `Count` as a child record rather than a property.
- No `PropertyTemplate` definitions.
- Unnamed mesh `Model`s (`"\x00\x01Model"`); Maya names the nodes itself (`transform1`, `_ncl1_1`).
- `TypedIndex` per `LayerElement`, second UV set as `LayerElementUV` index 1.

### 8.9 Regression checklist

1. `make -j8` clean.
2. `tris.iqm` (repo root) → FBX → headless Maya: **34 joints, 1 mesh, 50 stacks** (Blender: 34
   bones, 1 mesh, 50 actions).
3. An MD3-based model (e.g. Warfork `rlauncher.md3`) → FBX and GLB: Maya world bounding boxes must
   agree with the GLB to rounding, and the long axis must be the same in both.
4. Blender import (more forgiving, so necessary but not sufficient).
5. File starts with `Kaydara FBX Binary  `, reports version 7400, ends with the 176-byte footer.
6. When the refreshed lib is synced into `gskelview`, `tests/test_md3tags.exe` still passes.

Cheap structural check without a 3D app:

```python
import struct
d = open(path, 'rb').read()
assert d[:21] == b'Kaydara FBX Binary  ' and d[21:23] == b'\x1a\x00'
assert struct.unpack_from('<I', d, 23)[0] == 7400
assert d[-16:] == bytes([0xf8,0x5a,0x8c,0x6a,0xde,0xf5,0xd9,0x7e,
                          0xec,0xe9,0x0c,0xe3,0x75,0x8f,0x29,0x0b])
assert d[-176:-160] == bytes([0xfa,0xbc,0xab,0x09,0xd0,0xc8,0xd4,0x66,
                              0xb1,0x76,0xfb,0x83,0x1c,0xf7,0x26,0x7e])
```
