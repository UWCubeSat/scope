# SCOPE: Star Camera Optical Parameter Estimator

SCOPE is a system for satellites utilizing optical sensors (cameras) that re-calculates camera intrinsic and distortion parameters in-flight to preserve accurate measurement and ensure accurate distortion correction during image processing.

## Status

The full pipeline (dark frame, star centroiding, Levenberg-Marquardt fit) runs and recovers known calibrations from synthetic star images (`./synthetic-e2e.sh`). It has not yet been run on real images, and nothing downstream consumes its output yet.
