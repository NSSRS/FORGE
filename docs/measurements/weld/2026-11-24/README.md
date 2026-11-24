# V7 baseline and four-motor evidence — November 24

The operator selected physical **v7** as the working weld baseline and completed
the ENCOS motor-4 identification/four-motor test. This archive preserves original
source bytes; gzip changes storage only. [manifest.json](manifest.json) records
source paths, uncompressed and archived SHA-256 hashes. Local paths inside raw
reports are provenance. Original reports and older reviews are not rewritten.

| Evidence | File / interpretation |
| --- | --- |
| V7 complete live journal | [v7-session.jsonl.gz](v7-session.jsonl.gz), session `20261124T101206.334308Z-lz462pcc` |
| V7 final report | [v7-session.json.gz](v7-session.json.gz), including checked plan and final close |
| V7 reusable scheme and measurements | [Baseline](../../arm-weld-v7-baseline-2026-11-24.json); exact sampling, bounded A1/A6 response and original startup |
| V7 reproduction inputs | [Mapping](mapping.json), [standby](standby.json), [arm profile](arm-profile.json), [response profile](v7-response-profile.json) |
| Four ENCOS motors | [Original report](encos-four-motor.json.gz) and [review](../../encos-four-motor-physical-review-2026-11-24.json) |
| Publication verification | [Offline checks](../../publication-review-2026-11-24.json) |

V7's matched cycles 3-18 have 2.168 mm same-time tip RMS, 3.19 ms fitted tip lag
and 0.759 deg orientation peak. Whole-pass startup still peaks at 1.095 deg;
per-cycle stroke is 15.723-19.611 mm against 20 mm planned. The operator's
selection does not waive those limits. Independent tool-pose data remain absent.
The v8 startup experiment is historical evidence; v7 remains active and ILC deferred.

The physical ENCOS report passed +5/-5 RPM motion and below-0.5-RPM stop checks
on all four IDs, slots 0/1/3/4, with zero recorded faults or failed exchanges
and a closed owner. The operator separately reported normal visual behavior
after launching the four-motor keyboard/viewer session. Exact wheel location,
forward signs, gearing and load/host-loss qualification remain separate.

## Reproduce without hardware

From the repository root:

```bash
mkdir -p build/replay-20261124
gzip -dc docs/measurements/weld/2026-11-24/v7-session.jsonl.gz \
  > build/replay-20261124/v7-session.jsonl
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python scripts/analyze_weld_accuracy.py \
  build/replay-20261124/v7-session.jsonl \
  --orientation-target-deg 0.5 --max-orientation-deg 1.0
QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYTHONPATH=tests \
  .venv/bin/python -m unittest test_weld_response test_weld_command_sampling
```

The analyzer should report the missed whole-pass orientation maximum and manual
pause; exit 2 is expected for that recorded result. The regression replays all
561 v7 commands/references within 1e-10 degrees. It does not open a motor owner.
Current operator launch commands remain in [ARM_RIG_TEST.md](../../../ARM_RIG_TEST.md#weld-motion-accuracy-test)
and [CAN2_BENCH.md](../../../CAN2_BENCH.md#motor-4-identification-and-four-motor-test).
