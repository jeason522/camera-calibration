# Camera Intrinsic Calibration Tool (C++ / OpenCV)

A command-line tool that computes camera intrinsic parameters and distortion
coefficients from chessboard images. Supports both **standard** and
**fisheye** camera models.

## Results

| Model | RPE | k1 (distortion) |
|-------|-----|-----------------|
| Standard | **0.42 px** | -0.266 (barrel) |
| Fisheye  | **0.43 px** | equidistant projection |

Both models stay well below the 1.0 px quality threshold.
Standard model fits better for this dataset (non-fisheye lens).

### Before / After undistortion
![comparison](comparison.jpg)

### RPE per image — Standard model
![rpe standard](rpe_per_image.png)

### RPE per image — Fisheye model
![rpe fisheye](rpe_fisheye.png)

## Usage
```bash
./calibrate                          # standard model (default)
./calibrate --fisheye                # fisheye model (AVM cameras)
./calibrate --dir /path/to/images    # custom image directory
./calibrate --board 7x5              # custom board size
./calibrate --square 30.0            # custom square size (mm)
./calibrate --output result.yaml     # custom output path
```

## Pipeline
```
Chessboard images
      │
      ▼
findChessboardCorners()
cornerSubPix()            ← sub-pixel accuracy
      │
      ▼
calibrateCamera()         ← standard: polynomial distortion
fisheye::calibrate()      ← fisheye: equidistant projection
      │
      ▼
projectPoints()           ← compute per-image RPE
RPE bar chart             ← visualize quality per image
      │
      ▼
calib_result.yaml         ← K matrix + distortion coeffs
comparison.jpg            ← before / after undistort
```

## Build & Run
```bash
sudo apt install build-essential cmake libopencv-dev

git clone https://github.com/jeason522/camera-calibration.git
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
| `comparison.jpg` | Before / after undistortion |
| `rpe_per_image.png` | Per-image RPE — standard model |
| `rpe_fisheye.png` | Per-image RPE — fisheye model |

## Limitations

- Chessboard must be **fully visible** in each image (partial boards are rejected)
- At least **4 successful detections** are required for calibration
- `SQUARE_SIZE` is in millimeters; ensure it matches your physical board
- For fisheye lenses with very strong distortion, corners near image edges may not be detected

## Environment

- Language: C++17
- Library: OpenCV 4.6.0
- Build system: CMake
- Platform: Linux (WSL2 / Ubuntu 22.04)

## Related

- AVM Pipeline (uses this tool's output): [avm-pipeline](https://github.com/jeason522/avm-pipeline)