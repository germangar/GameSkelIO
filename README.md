# GameSkeIO

## 1. Project Overview
GameSekIO is a small model converter targetting typical skeletal animated models used in games.

## 2. Supported Formats
- **IQM**: Read/Write - Inter-Quake Model (Z-up, "Backward" CW winding).
- **GLB/GLTF**: Read/Write - GL Transmission Format (Y-up, Standard CCW winding).
- **FBX (Binary)**: Read/Write - Autodesk FBX (Y-up, Standard CCW winding). Native high-performance implementation.
- **SKM/SKP**: Read only - Warsow legacy skeletal format. (Z-up, "Backward" CW winding).

## 3. Internal Representation (Intermediate Model)
To maintain consistency across converters, all loaders/writers must adhere to:
- **Coordinate System**: Y-up (Right-Handed).
- **Winding Order**: Standard CCW (Counter-Clockwise).
- **Animation**: Time based. Sort of GLB format.
