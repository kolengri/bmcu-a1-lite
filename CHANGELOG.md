# Changelog

## 260b0404 — 2026-11-04

- Owner-reported completed two-color print on the development BMCU370C DM Pro/A1
  setup; the exact installed binary is released.
- Early filament presence during setup now pauses motion. Removal and stable
  empty inputs resume setup without a permanent filament-presence fault.
- The calibration-only completion cue is yellow while later setup remains pending.
- Local intake no longer generates an incorrect pull-PWM journal record, and
  diagnostic Flash submissions wait for local intake to become idle.
- Existing native Load/Unload, ownership, sensor and jam protections remain.
- 133 production-body host cases,13 profile guards and exact-BIN source replay
  passed; all1,079 padded blocks were verified during installation.
- Automatic USB log retrieval is still unavailable. Bundled export tools remain
  historical0403 research tooling and must not be used against0404.

## 260b0403 — historical source publication

Normal printer startup and experimental ISP-selected USB export. Full firmware
comparison succeeded, but marker verification blocked export. Superseded by0404
for normal use; kept as a historical source and tool reference.

## 260b0401 — historical physical checkpoint

Owner-reported successful Unload after requested-slot feedback and journal
retention changes. This checkpoint did not yet include a completed print.
