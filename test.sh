#!/bin/bash

# Test script for mesh simplification

set -e

echo "Building solution..."
g++ -O2 -std=c++17 baseline.cpp -o baseline

echo ""
echo "Testing with sample input..."
./baseline < sample.in > sample.out

echo ""
echo "Sample Input:"
head -3 sample.in

echo ""
echo "Sample Output:"
head -3 sample.out

# Calculate compression rate
input_verts=$(head -1 sample.in | awk '{print $1}')
output_verts=$(head -1 sample.out | awk '{print $1}')
compression=$(echo "scale=4; 1 - $output_verts / $input_verts" | bc)

echo ""
echo "Statistics:"
echo "  Input vertices:  $input_verts"
echo "  Output vertices: $output_verts"
echo "  Compression:     $(echo "$compression * 100" | bc)%"

echo ""
echo "✓ Test completed successfully!"
