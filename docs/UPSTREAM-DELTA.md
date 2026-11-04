# Why a standalone repository

Compare build260b0404 against upstream commit
`5b8e235576f9d6c30403e737451ef60f97857dac` in
[BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/tree/5b8e235576f9d6c30403e737451ef60f97857dac).

The normalized `src/**` and `platformio.ini` difference is **29 changed files,
5,518 added lines and553 deleted lines**. Line endings are normalized before
comparison. Tests, logs, binary assets, copied investigation work areas and the
additional linker scripts are excluded. [Per-file counts](UPSTREAM-DELTA.json).
These figures describe the accumulated implementation, not29 independent bugs.

The change spans AMS Lite identity and replies, motion/ownership handling,
calibration recovery, encoder behavior, boot selection, Flash layout and continuous
diagnostics. It is too broad for a focused upstream fix PR. This public repository
keeps the integrated working image, source and evidence explanation reviewable;
no large PR is opened against the original firmware repository. A future small
upstream contribution would need its own isolated behavior and regression proof.

Private device captures, hardware IDs, job records and local debug artifacts are
not included. Existing attribution and source notices are retained. This project
has not inferred a license grant from the original repository's public visibility.
