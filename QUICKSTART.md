# Quick Start Guide

## 🚀 Getting Started in 3 Steps

### 1. Build the Solution

```bash
g++ -O2 -std=c++17 baseline.cpp -o baseline
```

### 2. Run on Input

```bash
./baseline < your_mesh.in > your_mesh.out
```

### 3. Analyze Results

```bash
python3 analyze.py your_mesh.in your_mesh.out
```

## 📁 Project Structure

```
imc-challenge-huawei/
├── baseline.cpp         # Main solution (QEM algorithm)
├── baseline             # Compiled executable
├── Eigen/               # Linear algebra library (auto-downloaded)
├── README.md            # Problem overview
├── SOLUTION.md          # Detailed algorithm documentation
├── QUICKSTART.md        # This file
├── test.sh              # Test script
├── analyze.py           # Mesh validation tool
├── sample.in            # Sample input (cube with extra vertex)
└── sample.out           # Sample output (after simplification)
```

## 🧪 Testing

### Run Sample Test

```bash
./test.sh
```

Expected output:
```
Building solution...
Testing with sample input...
Statistics:
  Input vertices:  9
  Output vertices: 8
  Compression:     11.11%
✓ Test completed successfully!
```

### Validate Output

```bash
python3 analyze.py sample.in sample.out
```

This checks:
- ✓ Manifold topology (watertight, closed)
- ✓ No degenerate faces
- ✓ Compression statistics
- ⚠️ Hausdorff and SSIM (evaluated by system)

## ⚙️ Tuning the Algorithm

Edit `baseline.cpp`, find the `simplify()` function:

```cpp
static void simplify() {
    simplify_qem(0.5);  // 50% vertex reduction
}
```

### Conservative (High Quality)
```cpp
simplify_qem(0.7);  // Keep 70% of vertices
```

### Aggressive (High Compression)
```cpp
simplify_qem(0.3);  // Keep only 30% of vertices
```

### Finding Optimal Value

1. Start conservative: `simplify_qem(0.7)`
2. Submit and check SSIM score
3. If SSIM ≥ 0.95: Try more aggressive (e.g., 0.5)
4. If SSIM < 0.95: Try less aggressive (e.g., 0.8)
5. Binary search until you find the sweet spot

## 📊 Performance Tips

### For Large Meshes (1M+ vertices)

Compile with optimizations:
```bash
g++ -O3 -march=native -std=c++17 baseline.cpp -o baseline
```

### Memory Usage

The algorithm is memory-efficient:
- **Space**: O(V + F) for mesh + O(V) for quadrics
- **Time**: O(E log E) where E ≈ 3V
- **Example**: 1M vertices ≈ 200MB RAM, ~10-30 seconds

### Profiling

```bash
# Time execution
time ./baseline < large_mesh.in > large_mesh.out

# Memory usage (macOS)
/usr/bin/time -l ./baseline < large_mesh.in > /dev/null
```

## 🐛 Troubleshooting

### Compilation Error: "Eigen/Dense not found"

The Eigen library should be auto-included. If missing:
```bash
curl -L https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -o eigen.tar.gz
tar -xzf eigen.tar.gz
mv eigen-3.4.0/Eigen .
rm -rf eigen-3.4.0 eigen.tar.gz
```

### Output is Empty or Mesh is Invalid

Check validation with:
```bash
python3 analyze.py input.obj output.obj
```

Common issues:
- **Non-manifold**: Increase `target_ratio` (less aggressive)
- **Degenerate faces**: Bug in collapse validation (check code)
- **Wrong format**: Ensure input follows format (first line: `V F`)

### SSIM Score Too Low

The mesh was simplified too aggressively:
1. Increase target ratio in `simplify()`
2. Recompile and rerun
3. Consider implementing feature preservation (see SOLUTION.md)

### Hausdorff Distance Too Large

Rare with QEM algorithm, but if it happens:
- Reduce target ratio
- Implement boundary/feature edge detection
- Weight quadrics by local curvature

## 📚 Documentation

- **README.md**: Problem description, competition rules
- **SOLUTION.md**: Detailed algorithm explanation, tuning guide
- **Code comments**: Implementation details in baseline.cpp

## 🎯 Submission Checklist

Before submitting:

- [ ] Compiles without errors
- [ ] Runs successfully on sample input
- [ ] Output is valid (use `analyze.py`)
- [ ] Tuned for target compression rate
- [ ] Tested on all available test cases
- [ ] Output file < 256 MiB

## 🏆 Competition Strategy

1. **Submit baseline early** - Understand scoring system
2. **Profile test cases** - Some may be easier than others
3. **Per-case tuning** - Different meshes need different ratios
4. **Feature preservation** - Protect visually important regions
5. **Iterate quickly** - Fast compile-test-submit cycle

## 💡 Next Steps

### Immediate Improvements
- Implement optimal vertex placement (solve linear system)
- Add feature edge detection (normal angle threshold)
- Weight by curvature (protect high-curvature regions)

### Advanced Optimizations
- View-dependent simplification (6 camera views)
- Perceptual weighting (visibility, saliency)
- Adaptive stopping (estimate SSIM during simplification)

See **SOLUTION.md** for implementation details!

---

**Good luck with the competition! 🚀**
