# SCOPE: known gaps

Written 2026-10-09, describing the code as of the `star-centroid` branch (commit `0c9b42f`). At that commit the build, all 48 tests, lint, the `FLOAT_MODE` build and valgrind pass, but the program does not yet produce a calibration. This file lists what is missing so it is not lost once the branch is merged.

## Needed before the pipeline does real work

- **No real attitudes.** `src/scope/providers/factory.hpp` gives every star image an identity attitude (see the `TODO(lost-integration)` comment there). There is no flag or file for supplying per-image attitudes, so the star-centroid stage only does meaningful work in tests. Real attitudes from LOST must go through `LostAttitudeToScopeFrame` (`src/scope/projection/projection.hpp`) first.
- **Optimization is a stub.** `LMAOptimizationAlgorithm` (`src/scope/optimization/`) returns an empty vector. The Levenberg-Marquardt fit is the bulk of the remaining work.
- **No output.** `PrimaryScopePipelineExecutor::OutputResults` (`src/scope/command-line/execution/executors.cpp`) prints "Nothing is implemented :(" and discards the pipeline result. The output format is also undecided, because neither FOUND's nor LOST's `Camera` accepts distortion parameters yet.

## Behaviour to be aware of

- **CLI flags changed.** `--input-images` no longer exists. Use `--dark-frames` and `--star-images`. New flags: `--catalog-path`, `--centroid-threshold`, `--magnitude-threshold`.
- **The catalog file is required.** `--catalog-path` defaults to `./bright-star-catalog.tsv`. If the file is missing, `LoadBsc` throws, nothing catches it, and the program aborts. `download-bsc.sh` is meant to fetch the file but has not been tested since it was added; the `documentation/downloading-stars.md` it mentions does not exist in this repo.
- **Running with no arguments is not handled.** `main` in `src/scope/command-line/scope-main.cpp` checks `argc == 0` and then reads `argv[1]`, which is null when no arguments are given. The check should be `argc < 2`. Found by reading the code, not by running it; also present on `main`.
- **CLI and provider code has no tests.** `scope-main.cpp`, `parser.cpp`, `executors.cpp`, `converters.hpp`, `factory.hpp` and `stage-providers.hpp` are at 0% coverage. The algorithm modules (`catalog`, `projection`, `star-centroid`, `noise-filter`) are at 95–100%.

## Design issues still open in the star-centroid stage

- **Pixel-centre convention differs from LOST.** `ExtractCentroid` (`src/scope/star-centroid/coi.cpp`) treats integer coordinates as pixel centres. LOST adds 0.5 to its centroids, treating integers as pixel corners. Left alone, the half-pixel difference would be absorbed into the fitted principal point or attitude correction. Pick one convention and match what FOUND assumes.
- **Search window is fixed at 31 px.** `kRoiSize` in `src/scope/star-centroid/star-centroid.cpp` is a constant. How far a star can land from its predicted pixel depends on the lens and on how well the mounting rotation is known, so this should be an option.
- **Images are 8-bit.** `Image` is `unsigned char` (inherited from FOUND), so the dark frame, the dark subtraction and `--centroid-threshold` are all 8-bit. If the flight camera saves 10- or 12-bit frames, centroiding faint stars at 8 bits loses precision.
