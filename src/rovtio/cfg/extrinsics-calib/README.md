# Extrinsics Calibration Configuration

This configuration directory is set up for **online camera-IMU extrinsics calibration** using ROVTIO with stereo thermal cameras.

## Files

- `left-thermal-rovio2.yaml` - Left thermal camera intrinsics (from calibration set 2)
- `right-thermal-rovio2.yaml` - Right thermal camera intrinsics (from calibration set 2)
- `rovtio.phxpro.info` - ROVTIO configuration file (modified for extrinsics calibration)
- `rovtio.phxpro.launch.py` - Launch file for ROVTIO

## Configuration Changes for Extrinsics Calibration

### 1. Stereo Depth Initialization (Line 134)
```
doStereoInitialization true;
```
**Purpose:** Uses stereo matching between left and right cameras to initialize feature depth, providing better initial depth estimates than monocular initialization.

### 2. Extrinsics Calibration - Increased Covariances (Lines 70, 74)
```
vep 0.01;    # Linear extrinsics covariance [m^2] (was 0.0001, increased 100x)
vea 0.1;     # Rotational extrinsics covariance [rad^2] (was 0.01, increased 10x)
```
**Purpose:** Higher initial covariance allows the filter to **calibrate** the camera-IMU extrinsics online. Lower values would keep the extrinsics fixed at the provided initial values.

**Initial Extrinsics (from rovtio.phxpro.info):**
- Camera0 (left): Position = [-0.12, -0.04, -0.08] m, Rotation = [1, 0, 0, 0] (quaternion)
- Camera1 (right): Position = [0.12, -0.04, -0.08] m, Rotation = [1, 0, 0, 0] (quaternion)

### 3. Frame Visualization (Line 83)
```
doFrameVisualisation true;
```
**Purpose:** Enables visualization of camera frames with tracked features, useful for monitoring calibration quality in real-time.

## Camera Intrinsics (Calibration Set 2)

### Left Camera
- **Focal lengths**: fx = 407.75, fy = 407.92 pixels
- **Principal point**: cx = 313.51, cy = 242.85 pixels
- **Distortion (k1, k2, p1, p2, k3)**: [-0.381, 0.148, 0.0005, -0.0011, 0.0]

### Right Camera
- **Focal lengths**: fx = 412.39, fy = 413.01 pixels
- **Principal point**: cx = 307.01, cy = 241.57 pixels
- **Distortion (k1, k2, p1, p2, k3)**: [-0.393, 0.159, -0.0005, 0.0009, 0.0]

## Usage

To use this configuration, launch ROVTIO pointing to this config directory:

```bash
ros2 launch /external/smores_drone_software/src/rovtio/cfg/extrinsics-calib/rovtio.phxpro.launch.py
```

## Expected Behavior

1. **Feature depth** will be initialized using stereo matching when features are visible in both cameras
2. **Camera-IMU extrinsics** will be refined online as the system collects motion data
3. **Visualization** windows will show camera frames with tracked features for monitoring

## Monitoring Calibration

Watch for:
- Consistent feature tracking across both cameras
- Convergence of extrinsics parameters (should stabilize after sufficient excitation)
- Reduced reprojection errors over time

## Notes

- Ensure adequate motion excitation (rotation and translation) for extrinsics to converge
- Initial extrinsics should be reasonably accurate (within ~10cm position, ~10° rotation)
- Calibration quality depends on feature richness and motion diversity
