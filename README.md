# Zigzag Path Planner for Point Clouds

Pick four corners of a region on a 3-D point cloud and generate a dense
zigzag (snake) trajectory that covers the quadrilateral and follows the
surface geometry, including normals.

## Dependencies

- PCL ≥ 1.14
- Eigen3
- VTK (pulled in by PCL)
- CMake ≥ 3.5

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Usage

```bash
./main <input.pcd> [row_spacing]
```

| Argument | Default | Description |
|---|---|---|
| `input.pcd` | — | Point cloud file (required) |
| `row_spacing` | `0.05` | Distance between adjacent rows |

### Interactive controls

| Action | Effect |
|---|---|
| **Shift+click** on cloud | Pick a corner point (up to 4) |
| **Z** | Generate zigzag path and save `trajectory.txt` |
| **C** | Clear all picks and trajectory |

Pick the four corners of the region you want to cover in any order;
the tool sorts them counter-clockwise automatically.

## Output — `trajectory.txt`

One line per waypoint:

```
<index> <x> <y> <z> <nx> <ny> <nz>
```

`nx ny nz` is the estimated surface normal at that point.

