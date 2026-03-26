# Camera Intrinsic Calibration Tool (C++ / OpenCV)

A command-line tool that computes camera intrinsic parameters and distortion
coefficients from a set of chessboard images, following the Zhang (2000) method.

## Results

| Metric | Value |
|--------|-------|
| Reprojection Error (RPE) | **0.42 px** |
| fx / fy | 536.4 / 536.4 px |
| Principal point (cx, cy) | (342.1, 236.0) px |
| Distortion (k1) | -0.266 (barrel) |

![Before / After undistortion](comparison.jpg)

## Pipeline
```
Chessboard images
      │
      ▼
findChessboardCorners()   ← detect inner corners
cornerSubPix()            ← refine to sub-pixel accuracy
      │
      ▼
calibrateCamera()         ← solve for K and distortion coeffs
      │
      ▼
calib_result.yaml         ← intrinsic matrix + dist coeffs
comparison.jpg            ← before / after undistort visual
```

## Build & Run
```bash
# Requirements: CMake >= 3.10, OpenCV >= 4.x, g++ with C++17
sudo apt install build-essential cmake libopencv-dev

git clone https://github.com/YOUR_USERNAME/camera-calibration.git
cd camera-calibration
mkdir build && cd build
cmake ..
make
./calibrate
```

## Output files

| File | Description |
|------|-------------|
| `calib_result.yaml` | Camera matrix K and distortion coefficients |
| `comparison.jpg` | Side-by-side before/after undistortion |

## Environment

- Language: C++17
- Library: OpenCV 4.6.0
- Build system: CMake
- Platform: Linux (WSL2 / Ubuntu 22.04)