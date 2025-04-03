#!/bin/bash

# Script to convert all GLB files in a directory to DAE format using ASSIMP
# Usage: ./convert_glb_to_dae.sh [directory]

# Check if ASSIMP is installed
if ! command -v assimp &> /dev/null; then
    echo "Error: ASSIMP is not installed."
    echo "On Ubuntu/Debian, install it with: sudo apt install assimp-utils"
    echo "On macOS with Homebrew: brew install assimp"
    echo "On Windows with WSL or Git Bash, follow Ubuntu instructions"
    exit 1
fi

# Set directory to current directory or provided argument
DIR="${1:-.}"

# Check if directory exists
if [ ! -d "$DIR" ]; then
    echo "Error: Directory '$DIR' does not exist."
    exit 1
fi

# Count number of GLB files
GLB_COUNT=$(find "$DIR" -name "*.glb" -type f | wc -l)

if [ "$GLB_COUNT" -eq 0 ]; then
    echo "No GLB files found in '$DIR'"
    exit 0
fi

echo "Found $GLB_COUNT GLB files to convert in '$DIR'"
echo "Starting conversion..."

# Convert all GLB files to DAE
COUNTER=0
ERRORS=0

find "$DIR" -name "*.glb" -type f | while read -r file; do
    filename="${file%.glb}"
    echo -n "Converting $(basename "$file") to $(basename "${filename}.dae")... "
    
    if assimp export "$file" "${filename}.dae" > /dev/null 2>&1; then
        echo "Done"
        COUNTER=$((COUNTER + 1))
    else
        echo "Failed"
        ERRORS=$((ERRORS + 1))
    fi
done

echo ""
echo "Conversion complete!"
echo "Successfully converted: $COUNTER files"
if [ "$ERRORS" -gt 0 ]; then
    echo "Failed conversions: $ERRORS files"
fi
