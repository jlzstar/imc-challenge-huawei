# IMC Challenge Attempt: Perception-Aware Mesh Simplification

## Problem Overview

This challenge requires simplifying million-vertex 3D meshes while maintaining:
- **Visual fidelity**: SSIM score ≥ 0.95 across 6 camera views (normal map + depth map)
- **Geometric accuracy**: Hausdorff distance ≤ 5% of mesh diagonal
- **Topological validity**: Manifold (watertight, closed 2-manifold)
- **Maximum compression**: Minimize vertex count

## Solution Approach

### Algorithm: Quadric Error Metrics (QEM)

The implementation uses the classic Garland & Heckbert QEM algorithm with edge collapse operations:

1. **Quadric Matrix Computation**
   - For each face, compute a quadric matrix Q representing the plane equation
   - Accumulate quadrics at each vertex from adjacent faces
   - Quadric error = squared distance to all adjacent planes

2. **Edge Collapse Priority**
   - Build priority queue of edge collapses sorted by error cost
   - For edge (v0, v1), combined quadric Q = Q[v0] + Q[v1]
   - Collapse target = midpoint (optimal would solve for minimum error position)
   - Cost = quadric error at target position

3. **Manifold Preservation**
   - Link condition: Common neighbors of v0 and v1 must form exactly 2 triangles
   - No degenerate faces: After collapse, no faces with duplicate vertices
   - Updates adjacency structures after each collapse

4. **Iterative Simplification**
   - Collapse edges one by one until target vertex count reached
   - Update quadrics and add new edges after each collapse
   - Rebuild clean mesh from non-removed vertices/faces

### Key Features

- **Conservative**: Currently set to 50% reduction (can be tuned)
- **Topology-aware**: Validates manifold constraints before collapse
- **Efficient**: Priority queue for optimal collapse ordering
- **Robust**: Handles edge cases (removed vertices, face updates)

## Building & Running

```bash
# Compile (Eigen library included)
g++ -O2 -std=c++17 baseline.cpp -o baseline

# Run
./baseline < input.obj > output.obj
```

## Algorithm Parameters

Current settings in `simplify()`:
- `target_ratio = 0.5` (50% vertex reduction)

### Tuning Strategy

For competition optimization:
1. Start conservative (0.5)
2. Implement SSIM evaluation locally or via submission
3. Binary search for optimal ratio per test case
4. Potentially implement adaptive per-region simplification

## Future Enhancements

1. **Optimal Vertex Placement**
   - Solve Qv = b for minimal error position (not just midpoint)
   - Handle boundary/feature edges specially

2. **Perceptual Weighting**
   - Weight quadrics by view-dependent visibility
   - Preserve high-curvature regions (edge detection)
   - Penalize silhouette edges more heavily

3. **Adaptive Simplification**
   - Per-region error budgets
   - Protect visually salient features
   - Curvature-based thresholds

4. **Multi-resolution Strategy**
   - Hierarchical simplification
   - Progressive refinement based on SSIM feedback

5. **Normal Preservation**
   - Add penalty terms for normal deviation
   - Explicitly preserve sharp features

## References

- Garland & Heckbert (1997): "Surface Simplification Using Quadric Error Metrics"
- Hoppe (1996): "Progressive Meshes"
- SSIM: Wang et al. (2004): "Image Quality Assessment"

## Notes

- Input meshes are pre-normalized (centered, within unit sphere)
- Evaluation uses 6 orthogonal views at distance 2.5
- Rendering uses flat shading (per-face normals)
- Depth is perspective-correct interpolated
- Output limited to 256 MiB
