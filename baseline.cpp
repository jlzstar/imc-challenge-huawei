// Perception-Aware Lossless Simplification of 3D Meshes
// 
// Implementation: Quadric Error Metrics (QEM) with Edge Collapse
// Based on Garland & Heckbert (1997)
//
// Algorithm:
//   1. Compute quadric error matrix for each vertex (sum of adjacent face planes)
//   2. Build priority queue of edge collapses sorted by quadric error
//   3. Iteratively collapse edges while maintaining manifold topology
//   4. Rebuild clean mesh from remaining vertices/faces
//
// To compile and run:
//   g++ -O2 -std=c++17 baseline.cpp -o baseline
//   ./baseline < mesh.in > mesh.out

#include "Eigen/Dense"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

// Mesh representation. Rows are vertices/faces.
//   V : |vertices| x 3 matrix of (x, y, z) coordinates.
//   F : |faces|    x 3 matrix of 0-indexed vertex references (input is
//                  1-indexed; load_obj subtracts 1, save_obj adds it back).
using MeshV = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using MeshF = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;

static MeshV V;
static MeshF F;

// --- fast input -------------------------------------------------------------

static vector<char> slurp_stdin() {
    vector<char> buf;
    buf.reserve(1 << 27);
    char chunk[1 << 16];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), stdin)) > 0)
        buf.insert(buf.end(), chunk, chunk + n);
    buf.push_back('\0');
    return buf;
}

static void load_obj() {
    vector<char> buf = slurp_stdin();
    char* p = buf.data();

    long nv = strtol(p, &p, 10);
    long nf = strtol(p, &p, 10);
    V.resize(nv, 3);
    F.resize(nf, 3);

    for (long i = 0; i < nv; ++i) {
        // 'v'
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p;
        ++p;
        V(i, 0) = strtod(p, &p);
        V(i, 1) = strtod(p, &p);
        V(i, 2) = strtod(p, &p);
    }
    for (long i = 0; i < nf; ++i) {
        // 'f'
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p;
        ++p;
        F(i, 0) = (int)strtol(p, &p, 10) - 1;
        F(i, 1) = (int)strtol(p, &p, 10) - 1;
        F(i, 2) = (int)strtol(p, &p, 10) - 1;
    }
}


// --- fast output -----------------------------------------------------------------

// Print the mesh. Print 10 significant digits using %.10g for performance
static void save_obj() {
    string out;
    out.reserve((size_t)V.rows() * 40 + (size_t)F.rows() * 24 + 32);
    char line[96];

    out.append(line, snprintf(line, sizeof line, "%ld %ld\n",
                              (long)V.rows(), (long)F.rows()));
    for (Eigen::Index i = 0; i < V.rows(); ++i)
        out.append(line, snprintf(line, sizeof line, "v %.10g %.10g %.10g\n",
                                  V(i, 0), V(i, 1), V(i, 2)));
    for (Eigen::Index i = 0; i < F.rows(); ++i)
        out.append(line, snprintf(line, sizeof line, "f %d %d %d\n",
                                  F(i, 0) + 1, F(i, 1) + 1, F(i, 2) + 1));

    fwrite(out.data(), 1, out.size(), stdout);
}

// --- your implementation --------------------------------------------------------------

#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <algorithm>

using Vec3 = Eigen::Vector3d;
using Mat4 = Eigen::Matrix4d;

// Edge represented by sorted vertex indices
struct Edge {
    int v0, v1;
    Edge(int a, int b) : v0(min(a, b)), v1(max(a, b)) {}
    bool operator<(const Edge& e) const {
        return v0 < e.v0 || (v0 == e.v0 && v1 < e.v1);
    }
};

// Edge collapse with cost
struct Collapse {
    double cost;
    int v0, v1;
    Vec3 target;
    bool operator>(const Collapse& c) const { return cost > c.cost; }
};

// Compute quadric matrix for a plane (face)
static Mat4 plane_quadric(const Vec3& n, double d) {
    Mat4 Q = Mat4::Zero();
    Q(0, 0) = n.x() * n.x(); Q(0, 1) = n.x() * n.y(); Q(0, 2) = n.x() * n.z(); Q(0, 3) = n.x() * d;
    Q(1, 0) = n.y() * n.x(); Q(1, 1) = n.y() * n.y(); Q(1, 2) = n.y() * n.z(); Q(1, 3) = n.y() * d;
    Q(2, 0) = n.z() * n.x(); Q(2, 1) = n.z() * n.y(); Q(2, 2) = n.z() * n.z(); Q(2, 3) = n.z() * d;
    Q(3, 0) = d * n.x();     Q(3, 1) = d * n.y();     Q(3, 2) = d * n.z();     Q(3, 3) = d * d;
    return Q;
}

// Evaluate quadric error at a point
static double eval_quadric(const Mat4& Q, const Vec3& v) {
    return v.x() * (Q(0, 0) * v.x() + Q(0, 1) * v.y() + Q(0, 2) * v.z() + Q(0, 3))
         + v.y() * (Q(1, 0) * v.x() + Q(1, 1) * v.y() + Q(1, 2) * v.z() + Q(1, 3))
         + v.z() * (Q(2, 0) * v.x() + Q(2, 1) * v.y() + Q(2, 2) * v.z() + Q(2, 3))
         + (Q(3, 0) * v.x() + Q(3, 1) * v.y() + Q(3, 2) * v.z() + Q(3, 3));
}

// Build adjacency: vertex -> faces, vertex -> vertices
static void build_adjacency(
    vector<set<int>>& v2f,
    vector<set<int>>& v2v,
    vector<bool>& removed_faces,
    const MeshF& faces
) {
    int nv = (int)V.rows();
    v2f.assign(nv, set<int>());
    v2v.assign(nv, set<int>());
    removed_faces.assign(faces.rows(), false);
    
    for (int fi = 0; fi < faces.rows(); ++fi) {
        int i0 = faces(fi, 0), i1 = faces(fi, 1), i2 = faces(fi, 2);
        v2f[i0].insert(fi); v2f[i1].insert(fi); v2f[i2].insert(fi);
        v2v[i0].insert(i1); v2v[i0].insert(i2);
        v2v[i1].insert(i0); v2v[i1].insert(i2);
        v2v[i2].insert(i0); v2v[i2].insert(i1);
    }
}

// Check if edge collapse is valid (maintains manifoldness)
static bool is_collapse_valid(
    int v0, int v1,
    const vector<set<int>>& v2v,
    const vector<set<int>>& v2f,
    const vector<bool>& removed_faces,
    const MeshF& faces
) {
    // Check link condition: common neighbors must form exactly 2 triangles
    vector<int> common;
    for (int n : v2v[v0]) {
        if (v2v[v1].count(n)) common.push_back(n);
    }
    if (common.size() != 2) return false;
    
    // Check that we won't create degenerate faces
    for (int fi : v2f[v0]) {
        if (removed_faces[fi]) continue;
        int a = faces(fi, 0), b = faces(fi, 1), c = faces(fi, 2);
        // Skip faces that will be removed (contain both v0 and v1)
        if ((a == v0 || b == v0 || c == v0) && (a == v1 || b == v1 || c == v1)) continue;
        // Replace v0 with v1 and check if valid
        if (a == v0) a = v1;
        if (b == v0) b = v1;
        if (c == v0) c = v1;
        if (a == b || b == c || c == a) return false;
    }
    
    return true;
}

// Perform edge collapse
static void collapse_edge(
    int v0, int v1, const Vec3& target,
    vector<set<int>>& v2f,
    vector<set<int>>& v2v,
    vector<bool>& removed_faces,
    MeshF& faces,
    vector<bool>& removed_verts
) {
    // Update vertex position
    V(v0, 0) = target.x();
    V(v0, 1) = target.y();
    V(v0, 2) = target.z();
    
    // Mark v1 as removed
    removed_verts[v1] = true;
    
    // Remove faces containing both v0 and v1
    vector<int> faces_to_remove;
    for (int fi : v2f[v0]) {
        if (removed_faces[fi]) continue;
        int a = faces(fi, 0), b = faces(fi, 1), c = faces(fi, 2);
        if ((a == v1 || b == v1 || c == v1)) {
            faces_to_remove.push_back(fi);
        }
    }
    
    for (int fi : faces_to_remove) {
        removed_faces[fi] = true;
        int a = faces(fi, 0), b = faces(fi, 1), c = faces(fi, 2);
        v2f[a].erase(fi);
        v2f[b].erase(fi);
        v2f[c].erase(fi);
    }
    
    // Update all faces referencing v1 to reference v0
    vector<int> v1_faces(v2f[v1].begin(), v2f[v1].end());
    for (int fi : v1_faces) {
        if (removed_faces[fi]) continue;
        for (int j = 0; j < 3; ++j) {
            if (faces(fi, j) == v1) {
                faces(fi, j) = v0;
                v2f[v0].insert(fi);
            }
        }
        v2f[v1].erase(fi);
    }
    
    // Update adjacency
    for (int n : v2v[v1]) {
        if (n != v0) {
            v2v[v0].insert(n);
            v2v[n].erase(v1);
            v2v[n].insert(v0);
        }
    }
    v2v[v0].erase(v1);
    v2v[v1].clear();
}

// Simplify mesh using QEM
static void simplify_qem(double target_ratio) {
    int nv = (int)V.rows();
    int target_verts = max(8, (int)(nv * target_ratio));
    
    // Build adjacency
    vector<set<int>> v2f, v2v;
    vector<bool> removed_faces;
    build_adjacency(v2f, v2v, removed_faces, F);
    
    vector<bool> removed_verts(nv, false);
    
    // Compute quadric for each vertex
    vector<Mat4> Q(nv, Mat4::Zero());
    for (int fi = 0; fi < F.rows(); ++fi) {
        Vec3 v0 = V.row(F(fi, 0));
        Vec3 v1 = V.row(F(fi, 1));
        Vec3 v2 = V.row(F(fi, 2));
        Vec3 n = (v1 - v0).cross(v2 - v0).normalized();
        double d = -n.dot(v0);
        Mat4 Qf = plane_quadric(n, d);
        Q[F(fi, 0)] += Qf;
        Q[F(fi, 1)] += Qf;
        Q[F(fi, 2)] += Qf;
    }
    
    // Build edge collapse priority queue
    priority_queue<Collapse, vector<Collapse>, greater<Collapse>> pq;
    set<Edge> edges;
    
    for (int i = 0; i < nv; ++i) {
        for (int j : v2v[i]) {
            if (i < j) edges.insert(Edge(i, j));
        }
    }
    
    for (const Edge& e : edges) {
        Mat4 Qsum = Q[e.v0] + Q[e.v1];
        Vec3 target = (V.row(e.v0) + V.row(e.v1)) / 2.0;
        double cost = eval_quadric(Qsum, target);
        pq.push({cost, e.v0, e.v1, target});
    }
    
    // Perform collapses
    int active_verts = nv;
    while (!pq.empty() && active_verts > target_verts) {
        Collapse c = pq.top();
        pq.pop();
        
        if (removed_verts[c.v0] || removed_verts[c.v1]) continue;
        if (!is_collapse_valid(c.v0, c.v1, v2v, v2f, removed_faces, F)) continue;
        
        // Perform collapse
        collapse_edge(c.v0, c.v1, c.target, v2f, v2v, removed_faces, F, removed_verts);
        Q[c.v0] += Q[c.v1];
        active_verts--;
        
        // Add new edges
        for (int n : v2v[c.v0]) {
            Mat4 Qsum = Q[c.v0] + Q[n];
            Vec3 target = (V.row(c.v0) + V.row(n)) / 2.0;
            double cost = eval_quadric(Qsum, target);
            pq.push({cost, c.v0, n, target});
        }
    }
    
    // Rebuild mesh
    map<int, int> old2new;
    int new_idx = 0;
    for (int i = 0; i < nv; ++i) {
        if (!removed_verts[i]) {
            old2new[i] = new_idx++;
        }
    }
    
    MeshV newV(new_idx, 3);
    for (const auto& p : old2new) {
        newV.row(p.second) = V.row(p.first);
    }
    
    vector<Eigen::Vector3i> newF;
    for (int fi = 0; fi < F.rows(); ++fi) {
        if (removed_faces[fi]) continue;
        int a = F(fi, 0), b = F(fi, 1), c = F(fi, 2);
        if (removed_verts[a] || removed_verts[b] || removed_verts[c]) continue;
        a = old2new[a]; b = old2new[b]; c = old2new[c];
        if (a == b || b == c || c == a) continue;
        newF.push_back({a, b, c});
    }
    
    V = newV;
    F.resize(newF.size(), 3);
    for (size_t i = 0; i < newF.size(); ++i) {
        F.row(i) = newF[i];
    }
}

// Optimize the mesh: replace V and F
static void simplify() {
    // Conservative simplification - start with 50% reduction
    // In practice, you'd want to measure SSIM and adjust
    simplify_qem(0.5);
}

int main() {
    load_obj();
    simplify();
    save_obj();
    return 0;
}