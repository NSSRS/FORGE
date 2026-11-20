# Arm planner benchmark

The benchmark uses the actual MuJoCo arm geometry and candidate calibration,
without opening a serial port. Planning latency and preparation motion duration
are separate measurements. A successful path can still miss the requested
**five-second preparation goal**; the report counts these separately. Welding
duration is not included in that preparation goal.

**Release status: sim and actual motor test pending.** These automated model
experiments do not replace interactive simulator review or actual motor tests.

## November 20 measurements

The initial corpus contains 48 accepted starts from 91 draws: 24 around standby
and 24 across the raw joint ranges. The improved planner accepted **47/48**;
`broad-007` exceeded the 180-second planning budget. The original run was
interrupted after 14 complete measurements, so the paired comparison uses only
those 14 starts. Both runs used four processes and identical motion settings.

| Paired measure | Original | Improved |
| --- | ---: | ---: |
| Median planning latency | 73.10 s | 24.80 s |
| Median nominal preparation | 9.83 s | 8.48 s |
| Preparation within five seconds | 0/14 | 0/14 |

Median planning latency fell **66%**; nominal preparation fell **14%** on this
paired set. Across all 48 attempts, latency median/p95/max was
**18.68 / 83.09 / 180.00 s**, including the timeout. The batch took 377.82 s.
Accepted nominal preparation ranged from **6.20 to 29.57 s**; none met five
seconds. The bounded joint-order recovery remains expensive on difficult starts:
one accepted case spent 138 s in staging search, another 97 s in requested
standby recovery. The timeout is a benchmark failure, not a proven unreachable pose.

A separate, predeclared 16-start work-area sample used seed 20261121 and uniform
raw perturbations of ±[5, 2, 2, 6, 4, 6] degrees around the required raised pose,
with the same 2 mm start-clearance filter. **16/16** accepted plans met five
seconds nominally: **4.46–4.81 s**, median **4.68 s**. This sample remains separate
from the broad corpus. It shows that the starting region matters; it does not
establish a universal five-second route or physical tracking capability.

Final-controller replay passed for all **63 accepted plans**. The work-area
sample reached Ready in **4.58–4.94 s** on ideal fake shafts, with all 16 within
five seconds; the broad/standby corpus took **6.34–29.80 s**. Dense unfiltered
collision-model checks also passed for all 63 plans and are recorded separately
from these controller timings.
One replay exposed a recovery-stop rounding error one representable value past
the path endpoint. The controller now resolves validated stop times to their
exact stationary knots within the existing tolerance. Planning code and timed
results are unchanged by this controller correction.

Release validation passed **223 weld regressions** and **30 shared arm/GUI/model
tests**. The independent geometry pass covered 172,749 sampled configurations.
Checkpoint smoke checks confirmed that matching results are reused and changed
settings are rejected. These checks used no motor hardware.

The same four starts, settings and optimized planner gave:

| Execution | Batch wall time | Median request latency |
| --- | ---: | ---: |
| Serial | 88.90 s | 20.84 s |
| Two threads | 116.97 s | 58.47 s |
| Two processes | 53.96 s | 26.62 s |

Two processes improved batch throughput; threads were slower than serial in
this experiment. A single GUI request does not gain that batch throughput, so
the GUI retains its isolated persistent planner process and benefits from reuse
and route pruning. These are single-run workstation measurements, with cold
initialization and subsequent cache hits included, not a universal scaling claim.

On those same four starts with two processes, exhaustive staging took **94.95 s**
versus **53.96 s** with optimistic pruning, a 43% batch reduction. Both selected
the same staging poses and nominal preparation durations in every case. This
checks the pruning result against the retained exhaustive comparison mode.

The [curated measurement summary](measurements/arm-planner-benchmark-2026-11-20.json)
records source/artifact identities and pending acceptance. Complete local reports
remain under `build/planner-benchmark/2026-11-20/`.

## Reproduce

Use a reviewed mapping and its matching standby. Generate the corpus once, then
reuse that exact file for paired comparisons:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python scripts/benchmark_weld_planner.py \
  --generate --count 48 --seed 20261120 \
  --mapping build/arm_tests/hold-01-20261119/mapping.json \
  --standby build/arm_tests/hold-01-20261119/standby.json \
  --corpus build/planner-benchmark/corpus.json

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python scripts/benchmark_weld_planner.py \
  --corpus build/planner-benchmark/corpus.json \
  --executor processes --workers 2 --prep-seconds 5 \
  --output build/planner-benchmark/optimized
```

These mapping/standby paths are local evidence, not tracked fixtures. The corpus
embeds the mapping, standby, model fingerprint and random seed. Half the accepted
starts are near standby; half are drawn broadly inside raw joint bounds. Every
start must pass 2 mm model clearance. Rejection sampling is not a claim of uniform
free-space coverage. Every case targets the same complete weld motion.

The offline benchmark explicitly requests 100 deg/s preparation, bounded by the
existing motor command caps, and retains 30 deg/s² acceleration, 30 mm approach
lift and 2.3 s final lowering. This does not change the GUI's 5 deg/s default or
authorize that speed on hardware. Use `--max-speed-deg-s` and
`--lowering-seconds` for clearly labeled comparisons.

For a smaller experiment, add `--limit 8`. Compare `--executor serial`,
`--executor threads --workers 4` and `--executor processes --workers 4` using
different output directories. Use `--preparation-planner staged` to check every
staging candidate instead of pruning candidates that cannot beat the best route.
The recovery-only workload is available with `--stage recovery`.
Four workers approached the memory capacity of this 16 GiB machine; the recipe
uses two. The GUI retains one isolated persistent planning process.

Each case writes its result and accepted plan immediately. An interrupted run
can continue with the same command and `--resume`; changed code, corpus or
settings are rejected. Resume wall time covers only that invocation. Per-case
planning times retain the measurements from their original invocation. Keep
other tests and benchmarks idle during timing comparisons.

`--verify` independently rechecks accepted trajectories using the unfiltered
collision model, dense time-based sampling, derivative/raw bounds and the real
controller on ideal fake shafts. It runs after the planning timer stops, but
competes for CPU with other workers, so use a separate verification run for
clean latency comparisons. Neither replay nor model clearance qualifies load,
tracking, stopping, thermal behavior or HOLD-01.

The default 180 s per-case planning deadline applies to serial and process jobs
on systems supporting interval timers. Thread jobs cannot safely install a
per-thread signal timer; each row records whether the deadline was enforced.
Failed and timed-out cases remain in the latency distribution and success count.

Generated plans, case checkpoints, profiles and full reports belong under
ignored `build/planner-benchmark/`. Source hashes in the report identify the
measured implementation. Cache hits and phase timings distinguish a cold first
request from repeated requests in a persistent planner process.
