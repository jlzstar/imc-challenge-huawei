#!/usr/bin/env python3
"""
Analyze mesh simplification results.
Usage: python3 analyze.py input.obj output.obj
"""

import sys
import math

def parse_obj(filename):
    """Parse simplified OBJ format."""
    vertices = []
    faces = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
        
    # First line: vertex_count face_count
    header = lines[0].strip().split()
    nv, nf = int(header[0]), int(header[1])
    
    for line in lines[1:]:
        parts = line.strip().split()
        if not parts:
            continue
            
        if parts[0] == 'v':
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == 'f':
            faces.append([int(parts[1])-1, int(parts[2])-1, int(parts[3])-1])
    
    return vertices, faces

def compute_bbox(vertices):
    """Compute axis-aligned bounding box."""
    if not vertices:
        return None
    
    min_x = min(v[0] for v in vertices)
    max_x = max(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    max_y = max(v[1] for v in vertices)
    min_z = min(v[2] for v in vertices)
    max_z = max(v[2] for v in vertices)
    
    dx = max_x - min_x
    dy = max_y - min_y
    dz = max_z - min_z
    diagonal = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    return {
        'min': [min_x, min_y, min_z],
        'max': [max_x, max_y, max_z],
        'size': [dx, dy, dz],
        'diagonal': diagonal
    }

def validate_manifold(vertices, faces):
    """Check if mesh is manifold (each edge shared by exactly 2 faces)."""
    edges = {}
    
    for fi, face in enumerate(faces):
        for i in range(3):
            v0 = face[i]
            v1 = face[(i + 1) % 3]
            edge = tuple(sorted([v0, v1]))
            
            if edge not in edges:
                edges[edge] = []
            edges[edge].append(fi)
    
    non_manifold = []
    boundary = []
    
    for edge, face_list in edges.items():
        if len(face_list) != 2:
            if len(face_list) == 1:
                boundary.append(edge)
            else:
                non_manifold.append((edge, len(face_list)))
    
    return {
        'is_manifold': len(non_manifold) == 0 and len(boundary) == 0,
        'non_manifold_edges': non_manifold,
        'boundary_edges': boundary,
        'total_edges': len(edges)
    }

def check_degenerate_faces(vertices, faces):
    """Check for degenerate (zero-area) faces."""
    degenerate = []
    
    for fi, face in enumerate(faces):
        if face[0] == face[1] or face[1] == face[2] or face[2] == face[0]:
            degenerate.append((fi, 'duplicate_vertices'))
            continue
        
        # Check area
        v0 = vertices[face[0]]
        v1 = vertices[face[1]]
        v2 = vertices[face[2]]
        
        # Cross product for area
        e1 = [v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]]
        e2 = [v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]]
        
        cross = [
            e1[1]*e2[2] - e1[2]*e2[1],
            e1[2]*e2[0] - e1[0]*e2[2],
            e1[0]*e2[1] - e1[1]*e2[0]
        ]
        
        area = 0.5 * math.sqrt(cross[0]**2 + cross[1]**2 + cross[2]**2)
        
        if area < 1e-10:
            degenerate.append((fi, f'zero_area: {area}'))
    
    return degenerate

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 analyze.py input.obj output.obj")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    print("=" * 70)
    print("MESH SIMPLIFICATION ANALYSIS")
    print("=" * 70)
    
    # Load meshes
    print("\n📂 Loading meshes...")
    in_verts, in_faces = parse_obj(input_file)
    out_verts, out_faces = parse_obj(output_file)
    
    # Basic statistics
    print("\n📊 BASIC STATISTICS")
    print("-" * 70)
    print(f"Input:   {len(in_verts):>8} vertices, {len(in_faces):>8} faces")
    print(f"Output:  {len(out_verts):>8} vertices, {len(out_faces):>8} faces")
    
    compression_v = 1.0 - len(out_verts) / len(in_verts)
    compression_f = 1.0 - len(out_faces) / len(in_faces)
    print(f"\nVertex compression:  {compression_v*100:>6.2f}%")
    print(f"Face compression:    {compression_f*100:>6.2f}%")
    
    # Bounding box
    print("\n📦 BOUNDING BOX")
    print("-" * 70)
    in_bbox = compute_bbox(in_verts)
    out_bbox = compute_bbox(out_verts)
    
    print(f"Input diagonal:   {in_bbox['diagonal']:.6f}")
    print(f"Output diagonal:  {out_bbox['diagonal']:.6f}")
    print(f"Max Hausdorff:    {in_bbox['diagonal'] * 0.05:.6f} (5% constraint)")
    
    # Manifold check
    print("\n🔍 TOPOLOGY VALIDATION")
    print("-" * 70)
    
    manifold = validate_manifold(out_verts, out_faces)
    
    if manifold['is_manifold']:
        print("✓ Mesh is manifold (watertight)")
    else:
        print("✗ Mesh is NOT manifold!")
        if manifold['boundary_edges']:
            print(f"  - {len(manifold['boundary_edges'])} boundary edges (open mesh)")
        if manifold['non_manifold_edges']:
            print(f"  - {len(manifold['non_manifold_edges'])} non-manifold edges")
            for edge, count in manifold['non_manifold_edges'][:5]:
                print(f"    Edge {edge}: {count} faces (should be 2)")
    
    print(f"\nTotal edges: {manifold['total_edges']}")
    print(f"Expected:    {len(out_faces) * 3 // 2} (for closed manifold)")
    
    # Degenerate faces
    degenerate = check_degenerate_faces(out_verts, out_faces)
    
    if degenerate:
        print(f"\n✗ Found {len(degenerate)} degenerate faces:")
        for fi, reason in degenerate[:5]:
            print(f"  Face {fi}: {reason}")
        if len(degenerate) > 5:
            print(f"  ... and {len(degenerate) - 5} more")
    else:
        print("\n✓ No degenerate faces")
    
    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    
    all_valid = manifold['is_manifold'] and len(degenerate) == 0
    
    if all_valid:
        print("✓ Output mesh is VALID")
        print(f"✓ Achieved {compression_v*100:.2f}% vertex reduction")
        print("\n⚠️  Still need to verify:")
        print("  - Hausdorff distance < 5% of diagonal")
        print("  - SSIM score ≥ 0.95 (system evaluation)")
    else:
        print("✗ Output mesh has VALIDATION ERRORS")
        print("\n⚠️  Fix issues before submission:")
        if not manifold['is_manifold']:
            print("  - Mesh must be manifold (closed, watertight)")
        if degenerate:
            print("  - Remove degenerate faces")
    
    print("=" * 70)

if __name__ == '__main__':
    main()
