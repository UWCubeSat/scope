# SCOPE: Star Camera Optical Parameter Estimator

SCOPE is a system for satellites utilizing optical sensors (cameras) that re-calculates camera intrinsic and distortion parameters in-flight to preserve accurate measurement and ensure accurate distortion correction during image processing.

## Status

The full pipeline (dark frame, star centroiding, Levenberg-Marquardt fit) runs and recovers known calibrations from synthetic star images (`./synthetic-e2e.sh`). It has not yet been run on real images, and nothing downstream consumes its output yet.

## Building

Build inside the devcontainer (`.devcontainer/`: Ubuntu 24.04 and GCC 13, the same as CI). From the repository root:

```
cmake -S . -B build
cmake --build build -j 4
./build/bin/scope-test
```

That builds `scope`, the synthetic data generator `scope-synth` and the tests into `build/bin/`. The tests must run from the repository root. Configure options:

- `-DCMAKE_BUILD_TYPE=RelWithDebInfo` for an optimized build. The default is `Debug`.
- `-DSANITIZE=OFF` builds the tests without AddressSanitizer and UndefinedBehaviorSanitizer, to run them under valgrind.
- `-DCOVERAGE=ON` instruments the tests. Run `gcovr` after them for a report in `build/coverage/`.

`./download-bsc.sh` fetches the star catalog `scope` reads, and `./synthetic-e2e.sh` runs the whole pipeline on a synthetic data set and checks the fit against the truth. CI (`.github/workflows/ci.yml`) runs all of that, the tests under valgrind, and `cpplint --recursive src test`.
