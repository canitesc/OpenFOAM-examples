#!/bin/bash

# OpenFOAM smoke plume simulation script

echo "Running smoke plume simulation..."

# Clean previous results
rm -rf processor* postProcessing

# Generate mesh
echo "Generating mesh..."
blockMesh

# Check mesh quality
echo "Checking mesh..."
checkMesh

# Run simulation
echo "Running simulation..."
buoyantPimpleFoam

echo "Simulation complete!"