#!/usr/bin/env bash
# Checks the whole pipeline on synthetic images, the way the observatory data
# will be checked: calibrate on some images from the datasheet prior, then hold
# the calibration fixed and measure the residuals on images the fit never saw
# (the paper's Fig. 7). Also compares the fit with the truth the images were
# rendered from, which real data cannot do.
#
# Usage: ./synthetic-e2e.sh [output-dir] [scope-synth flags...]
#
# Run inside the devcontainer after building scope and scope-synth
# (cmake -S . -B build && cmake --build build -j 4), with the catalog downloaded
# (./download-bsc.sh). FIT_IMAGES and HELD_OUT_IMAGES set the split (15 and 15),
# BIN where the binaries are (build/bin). Exits non-zero if the fit did not
# converge, a fitted parameter is more than 5 sigma from the truth, or the
# held-out residual RMS is 0.5 px or more.
set -euo pipefail

OUT=${1:-build/synthetic/e2e}
shift || true
BIN=${BIN:-build/bin}
FIT_IMAGES=${FIT_IMAGES:-15}
HELD_OUT_IMAGES=${HELD_OUT_IMAGES:-15}

# The Edmund #35-142 datasheet values on the IMX252 (reference/LENS.md §4).
PRIOR=(--focal-length-x 1739.13 --focal-length-y 1739.13 --principal-point-x 1031.5 --principal-point-y 771.5
       --alpha 0 --k1 -0.157 --k2 0.082 --k3 0 --p1 0 --p2 0)

echo "== Rendering $((FIT_IMAGES + HELD_OUT_IMAGES)) star images into $OUT"
"$BIN/scope-synth" --output-dir "$OUT" --star-images $((FIT_IMAGES + HELD_OUT_IMAGES)) "$@"

darks=$(printf '%s\n' "$OUT"/dark-*.pgm | paste -sd, -)
stars=("$OUT"/star-*.pgm)
fit_stars=$(printf '%s\n' "${stars[@]:0:FIT_IMAGES}" | paste -sd, -)
held_out_stars=$(printf '%s\n' "${stars[@]:FIT_IMAGES}" | paste -sd, -)
grep -v '^#' "$OUT/attitudes.txt" | head -n "$FIT_IMAGES" > "$OUT/fit-attitudes.txt"
grep -v '^#' "$OUT/attitudes.txt" | tail -n +$((FIT_IMAGES + 1)) > "$OUT/held-out-attitudes.txt"
catalog=$(sed -n 's/.*--catalog-path \([^ ]*\).*/\1/p' "$OUT/scope-args.txt")

echo "== Fitting $FIT_IMAGES images from the datasheet prior"
"$BIN/scope" run --dark-frames "$darks" --star-images "$fit_stars" --attitudes "$OUT/fit-attitudes.txt" \
    --catalog-path "$catalog" "${PRIOR[@]}" | tee "$OUT/fit.txt"

# The fit prints its parameters as flags on the as-prior line.
read -r -a fitted <<< "$(sed -n 's/^ *as-prior: //p' "$OUT/fit.txt")"

echo "== Holding the calibration fixed on $HELD_OUT_IMAGES held-out images"
"$BIN/scope" run --dark-frames "$darks" --star-images "$held_out_stars" \
    --attitudes "$OUT/held-out-attitudes.txt" --catalog-path "$catalog" "${fitted[@]}" \
    --fixed-parameters all | tee "$OUT/held-out.txt"

echo "== Fit against the truth"
awk '
    FNR == NR {
        if ($1 == "misalignment:") { for (i = 0; i < 3; ++i) misalignment[i] = $(i + 2) }
        else if ($1 ~ /:$/ && $1 != "true-attitude:") { truth[$1] = $2 }
        next
    }
    $1 == "converged:" { converged = $2 }
    $1 in truth {
        name = substr($1, 1, length($1) - 1)
        if ($3 != "+/-") { printf "%-18s %14.8g %14.8g  (fixed)\n", name, truth[$1], $2; next }
        z = ($2 - truth[$1]) / $4
        printf "%-18s %14.8g %14.8g +/- %-10.3g %6.2f sigma\n", name, truth[$1], $2, $4, z
        if (z > 5 || z < -5) { bad = bad " " name }
    }
    $1 == "alignment-correction:" {
        printf "%-18s %8.2f %8.2f %8.2f arcsec, truth %8.2f %8.2f %8.2f\n", "alignment", $2, $3, $4,
               misalignment[0], misalignment[1], misalignment[2]
    }
    END {
        if (converged != "yes") { print "FAIL: the fit did not converge"; exit 1 }
        if (bad != "") { print "FAIL: more than 5 sigma from the truth:" bad; exit 1 }
    }
' "$OUT/truth.txt" "$OUT/fit.txt"

awk '
    $1 == "residual-rms:" {
        printf "held-out residual RMS: %s px (must be under 0.5)\n", $2
        if ($2 + 0 >= 0.5) { print "FAIL: held-out residuals too large"; exit 1 }
    }
' "$OUT/held-out.txt"
echo "PASS"
