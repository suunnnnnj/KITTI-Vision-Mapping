# KITTI Vision Mapping

This project generates a colored point cloud map from the KITTI dataset. It uses the dataset's color images and pose information to project points into a 3D world coordinate system and export the result as a LAS file.

## Table of Contents

- [Dataset Structure](#dataset-structure)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Running](#running)
- [Module Overview](#module-overview)
- [Testing](#testing)
- [License](#license)

## Dataset Structure

The KITTI dataset should be organized as follows:

```
<kitti_root>/
├── image_02/           # Left color images
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
└── poses.txt           # ASCII file with one 3x4 pose matrix per line
```

- **image_02/**: Contains color images (`.png`).
- **poses.txt**: Each line has 12 values (row-major 3x4 transform) corresponding to each image frame.

## Project Structure

```
.
├── CMakeLists.txt      # Top-level build configuration
├── cmake/              # Custom CMake modules (FindXXX)
├── include/            # Public headers
│   ├── io/             # Image and pose loaders
│   ├── core/           # Mapper interface
│   └── utils/          # Utility functions
├── src/                # Implementation files
│   ├── io/
│   ├── core/
│   └── utils/
├── tests/              # Unit tests (GoogleTest)
├── examples/           # Example usage scripts
└── README.md           # This file
```

## Prerequisites

- CMake ≥ 3.12
- C++14 compiler
- [PCL](https://pointclouds.org/) (Point Cloud Library)
- [libLAS](https://liblas.org/)
- OpenCV
- Boost (filesystem)
- OpenMP (optional)

On Ubuntu, you can install dependencies via:

```bash
sudo apt-get update
sudo apt-get install cmake libpcl-dev liblas-dev libopencv-dev libboost-filesystem-dev libomp-dev
```

## Building

```bash
git clone <repo_url>
cd KITTI-Vision-Mapping
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The executable `kitti_mapper` will be generated in `build/`.

## Running

```bash
./kitti_mapper \
  --img_dir <kitti_root>/image_02 \
  --img_pattern "%06d.png" \
  --pose_file <kitti_root>/poses.txt \
  --output out.las
```

- `--img_dir`: Path to color images directory.
- `--img_pattern`: `printf`-style pattern for filenames.
- `--pose_file`: Path to pose text file.
- `--output`: Name of the output LAS file.

Example:

```bash
./kitti_mapper --img_dir ../data/image_02 --img_pattern "%06d.png" --pose_file ../data/poses.txt --output map.las
```

## Module Overview

- **io**: `ImageLoader`, `PoseLoader`
- **core**: `Mapper` (main pipeline)
- **utils**: Math functions, filesystem helpers, progress bar

Refer to headers in `include/` for detailed API.

## Testing

Unit tests use GoogleTest. To build and run tests:

```bash
cd build
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)
ctest --output-on-failure
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.


