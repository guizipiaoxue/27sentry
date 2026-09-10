# Third-party source

## GTSAM

- Upstream: https://github.com/borglab/gtsam
- Vendored commit: `067ca8d0c55ffec619d57ee48b6ba0bdcfcb7ce2`
- Source version: `4.3a2`
- Retrieved branch: `develop`
- License: BSD-3-Clause; see `gtsam/LICENSE.BSD` and `gtsam/LICENSE`

The source is stored directly in `gtsam/` rather than as a Git submodule so a
clone of this repository is self-contained. Local build products and the
upstream repository's nested `.git` metadata are intentionally excluded. Three
upstream `.gitignore` files contain explicit exceptions so files that were
already tracked upstream remain visible to a normal `git add` after vendoring.

`odom/src/loop_closure/CMakeLists.txt` builds only the GTSAM core required by
the iSAM2 backend. Tests, examples, Python, unstable APIs, TBB integration and
native-CPU code generation are disabled. The vendored build is statically
linked into the ROS node to avoid a runtime dependency on an external GTSAM
installation.
