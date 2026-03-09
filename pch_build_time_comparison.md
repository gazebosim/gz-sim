# Precompiled Headers (PCH) Build Time Comparison

## Purpose

Measure the test compilation time improvement introduced by the `jrivero/pch`
branch (commit `edaf8f811`) which implements precompiled headers for gz-sim
test targets.

## Environment

- Build type: `RelWithDebInfo`
- Parallelism: `-j5`
- Library code: already built (not cleaned between runs)
- Only test artifacts were cleaned and rebuilt

## Procedure

### 1. Build tests on main branch (baseline)

```bash
git checkout main

# Clean only test artifacts (object files and binaries)
find build/src -path "*/CMakeFiles/UNIT_*" -type d -exec rm -rf {} +
find build/src -path "*/CMakeFiles/INTEGRATION_*" -type d -exec rm -rf {} +
find build/test -path "*/CMakeFiles/UNIT_*" -type d -exec rm -rf {} +
find build/test -path "*/CMakeFiles/INTEGRATION_*" -type d -exec rm -rf {} +
rm -f build/bin/UNIT_* build/bin/INTEGRATION_*

# Reconfigure (needed after branch switch since CMakeLists.txt changed) and rebuild
cd build && cmake ..
time cmake --build . -j5
```

### 2. Build tests on PCH branch

```bash
git checkout jrivero/pch

# Same cleanup of test artifacts
find build/src -path "*/CMakeFiles/UNIT_*" -type d -exec rm -rf {} +
find build/src -path "*/CMakeFiles/INTEGRATION_*" -type d -exec rm -rf {} +
find build/test -path "*/CMakeFiles/UNIT_*" -type d -exec rm -rf {} +
find build/test -path "*/CMakeFiles/INTEGRATION_*" -type d -exec rm -rf {} +
rm -f build/bin/UNIT_* build/bin/INTEGRATION_*

# Reconfigure and rebuild
cd build && cmake ..
time cmake --build . -j5
```

## Results

| Metric | Main Branch | PCH Branch | Improvement |
|--------|-------------|------------|-------------|
| **Real (wall clock)** | 4m 27.9s | 3m 24.3s | 63.6s faster (23.7%) |
| **User (CPU)** | 19m 18.4s | 14m 58.3s | 4m 20.1s faster (22.5%) |
| **Sys** | 2m 15.5s | 1m 44.4s | 31.1s faster (22.9%) |

## Conclusion

The PCH branch reduces test compilation wall-clock time by approximately 24%,
saving over a minute on a `-j5` build. CPU time savings are consistent across
user and sys, confirming the improvement comes from reduced redundant header
parsing.
