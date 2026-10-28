# Ubuntu 24.04, as on the CI runners, so both build with the same GCC.
FROM ubuntu:24.04

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential ca-certificates clang-format cmake curl git python3-pip valgrind \
    && pip install --no-cache-dir --break-system-packages cpplint==2.0.0 gcovr==8.3 \
    && rm -rf /var/lib/apt/lists/*
