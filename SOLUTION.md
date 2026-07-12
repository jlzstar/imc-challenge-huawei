# Solution Guide: Perception-Aware Mesh Simplification

## Quick Start

```bash
# Compile
g++ -O2 -std=c++17 baseline.cpp -o baseline

# Run on input file
./baseline < input.obj > output.obj

# Test with sample
./test.sh
```

## Implementation Details

### Core Algorithm: Quadric Error Metrics (QEM)

The solution implements the classic mesh simplification algorithm using edge collapses guided by quadric error metrics.

#### Step 1: Quadric Matrix Computation

For each triangular face with normal **n** = (nx, ny, nz) and plane equation **n**·**p** + d = 0:

```
Q = | nx²   nx·ny  nx·nz  nx·d |
    | ny·nx ny²    ny·nz  ny·d |
    | nz·nx nz·ny  nz²    nz·d |
    | d·nx  d·ny   d·nz   d²   |
```

Each vertex accumulates quadrics from all adjacent faces:
```
Q[vertex] = Σ Q[face] for all adjacent faces
```

#### Step 2: Edge Collapse Cost

For an edge (v0, v1), the collapse cost is computed as:
```
Q_combined = Q[v0] + Q[v1]
target = (v0 + v1) / 2          // Midpoint (simplified)
cost = target^T · Q_combined · target
```

Optimal placement would solve:
```
[ Q_11  Q_12  Q_13  Q_14 ] [ x ]   [ 0 ]
[ Q_21  Q_22  Q_23  Q_24 ] [ y ] = [ 0 ]
[ Q_31  Q_32  Q_33  Q_34 ] [ z ]   [ 0 ]
[ 0     0     0     1    ] [ 1 ]   [ 1 ]
```

Current implementation uses midpoint for robustness.

#### Step 3: Manifold Validation

Before collapsing edge (v0, v1), verify:

1. **Link Condition**: Common neighbors of v0 and v1 must be exactly 2 vertices
   - These form the two triangular faces sharing the edge
   - Ensures no topological holes or non-manifold configurations

2. **Non-degeneracy**: After replacing v0 with v1 in all faces:
   - No face can have duplicate vertices (a==b or b==c or c==a)
   - Prevents zero-area triangles

3. **Adjacency Consistency**: Update data structures correctly:
   - Remove faces containing both v0 and v1
   - Transfer v1's faces to v0
   - Update vertex-to-vertex adjacency

#### Step 4: Iterative Simplification

```cpp
while (active_vertices > target && !queue.empty()) {
    collapse = queue.pop();
    if (valid(collapse)) {
        perform_collapse(collapse);
        add_new_edge_collapses(queue);
    }
}
```

### Data Structures

**Adjacency Lists**:
- `v2f[v]`: Set of face indices containing vertex v
- `v2v[v]`: Set of vertex indices adjacent to v

**Tracking Removal**:
- `removed_verts[v]`: Boolean marking removed vertices
- `removed_faces[f]`: Boolean marking removed faces

**Quadric Storage**:
- `Q[v]`: 4x4 matrix storing accumulated error for vertex v

### Complexity Analysis

- **Preprocessing**: O(F) for quadric computation
- **Queue Building**: O(E log E) where E ≈ 3V for manifold meshes
- **Collapse Loop**: O(E log E) in worst case
- **Mesh Rebuild**: O(V + F)
- **Total**: O(E log E) ≈ O(V log V)

For million-vertex meshes, this is efficient enough.

## Tuning Parameters

### Current Settings

```cpp
static void simplify() {
    simplify_qem(0.5);  // Target 50% vertex reduction
}
```

### Recommended Tuning Strategy

1. **Conservative Start**: Begin with `target_ratio = 0.7` (30% reduction)
2. **Test Submission**: Check SSIM score
3. **Binary Search**:
   ```
   if SSIM < 0.95:  increase ratio (less aggressive)
   if SSIM ≥ 0.95:  decrease ratio (more aggressive)
   ```
4. **Per-Case Tuning**: Different meshes may have different optimal ratios

### Advanced Parameters (for future enhancement)

```cpp
// Weight quadrics by curvature
double curvature_weight = 1.0 + curvature[vertex];

// Penalize silhouette edges
if (is_silhouette_edge(e)) {
    cost *= silhouette_penalty;  // e.g., 10.0
}

// Boundary preservation
if (is_boundary_vertex(v)) {
    Q[v] *= boundary_weight;  // e.g., 100.0
}
```

## Validation Checklist

Before submitting, verify:

- [ ] Mesh is manifold (each edge shared by exactly 2 faces)
- [ ] No degenerate faces (all positive area)
- [ ] Vertex count: 1 ≤ V_out ≤ V_in
- [ ] All face indices valid (0 ≤ index < V_out)
- [ ] Output file size < 256 MiB
- [ ] Hausdorff distance < 5% of diagonal (evaluated by system)
- [ ] SSIM score ≥ 0.95 (evaluated by system)

## Debugging Tips

### Check Manifold Property

```bash
# Count edges (should be E = 3F/2 for closed manifold)
grep "^f" output.obj | wc -l    # F faces
# Each face has 3 edges, shared by 2 faces → E = 3F/2
```

### Visualize Output

Use MeshLab, Blender, or similar tools:
```bash
# Convert to standard OBJ for viewing
# (Add header: just vertex and face lines)
```

### Profile Performance

```bash
# Time execution
time ./baseline < large_mesh.in > /dev/null

# Memory usage
/usr/bin/time -l ./baseline < large_mesh.in > /dev/null
```

## Known Limitations

1. **Midpoint Placement**: Not optimal, can be improved by solving linear system
2. **Uniform Weighting**: Doesn't account for perceptual importance
3. **No Feature Detection**: Sharp edges and corners not explicitly preserved
4. **Fixed Target**: Doesn't adaptively stop at SSIM threshold
5. **No View-Dependent**: Doesn't consider the 6 evaluation views

## Future Improvements

### Priority 1: Better Vertex Placement
```cpp
// Solve for optimal position minimizing quadric error
Vec3 optimal_position(const Mat4& Q) {
    // Solve Q_3x3 * v = -q (using top-left 3x3 block)
    // If singular, fall back to edge midpoint or endpoint
}
```

### Priority 2: Feature Preservation
```cpp
// Detect and preserve sharp features
bool is_sharp_edge(int v0, int v1) {
    // Compare normals of adjacent faces
    // If angle > threshold, mark as feature edge
}
```

### Priority 3: Perceptual Weighting
```cpp
// Weight by visibility from evaluation cameras
for (int view = 0; view < 6; ++view) {
    visibility[vertex] += is_visible_from(vertex, camera[view]);
}
Q[vertex] *= (1.0 + visibility_weight * visibility[vertex]);
```

### Priority 4: Adaptive Simplification
```cpp
// Estimate SSIM incrementally and stop when threshold reached
while (estimated_ssim() > 0.95 && can_simplify()) {
    perform_next_collapse();
}
```

## References

- **[Garland97]** Garland, M. & Heckbert, P. (1997). "Surface Simplification Using Quadric Error Metrics". SIGGRAPH.
- **[Hoppe96]** Hoppe, H. (1996). "Progressive Meshes". SIGGRAPH.
- **[Hoppe97]** Hoppe, H. (1997). "View-Dependent Refinement of Progressive Meshes". SIGGRAPH.
- **[Wang04]** Wang, Z. et al. (2004). "Image Quality Assessment: From Error Visibility to Structural Similarity". IEEE TIP.

## Competition Tips

1. **Test Early**: Submit baseline to understand scoring
2. **Profile First**: Identify bottlenecks before optimizing
3. **Validate Locally**: Implement basic SSIM checker if possible
4. **Per-Case Strategy**: Different test cases may need different approaches
5. **Conservative Initially**: Start with less aggressive simplification
6. **Feature Preservation**: Identify and protect visually important regions
7. **Silhouette Awareness**: Edges on the visual boundary are most important
8. **Normal Consistency**: Large normal changes → visible artifacts

Good luck! 🚀
