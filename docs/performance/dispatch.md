# Dispatch optimization: measurements and validation

This is the first low-risk performance step from the architecture review. It removes repeated world scans without changing state ownership, integration formulas, object order, contact solving or observation timing. It does not introduce an ECS state store or a read-side service.

## Changes

- Individual behavior registrations can declare `requires_neighbors=False`. The four built-in dash handlers do so. Other handlers retain their live, role-filtered neighbor list unless they explicitly opt out.
- Requirements are associated with the registration and callable identity. Replacing a function or class under a dash name does not silently inherit dash's opt-out. Initialized class handlers retain their own requirement until reinitialization.
- Action targets use per-call, lazily built ID/name maps. N objects and K targets need O(N + K) lookup work, without persistent indexes to invalidate after deletion or reload. Default targets do not build an index.

## TDD and correctness

The initial test checkpoint is commit `b60d219`. With the original production code, the added dispatch tests produced **11 failures and 16 passes**. Failures demonstrated unnecessary neighbor collection, the missing capability declaration and quadratic identity/equality work. Existing custom-neighbor semantics already passed.

The completed change has 31 additional test cases. The focused dispatch/action set passes **35 tests**; the full repository suite passes **1200 tests**, with **52 skips** because `pyrvo` and `pynput` are not installed. Skips are not counted as verified optional integrations. The full run took 83.37 seconds on the development machine.

Coverage includes all four dash kinematics, custom function/class/factory handlers, role filtering, live sequential observations, registration replacement, class reload/fallback, mixed IDs/names, duplicate targets, first-match behavior, unknown targets and object deletion.

Across all 30 corresponding benchmark runs, the complete recorded trajectories and canonical final WorldState snapshots have identical SHA-256 digests between baseline and optimized versions. This checks intermediate motion as well as final state for these deterministic scenarios; it is not a proof for every possible plugin or numerical workload.

## End-to-end measurements

Baseline: upstream commit `39ed2f0e4ca006f5061d8808b19b46b2cfff2fca`, exported as a separate source tree. Both versions use the same interpreter, dependencies, scenes and benchmark driver. Each case uses 5 fresh environments, 10 warm-up steps and 200 timed steps per environment. JSON generation, trajectory hashing and setup are outside the step timer. This workflow did not launch tests or model inference concurrently with either measurement.

The table reports the median of five per-run step medians, in milliseconds.

| Scenario | Baseline ms | Optimized ms | Speedup | Step-time reduction |
| --- | ---: | ---: | ---: | ---: |
| dash_1 | 0.0503 | 0.0501 | 1.004x | 0.40% |
| dash_10 | 0.3536 | 0.3409 | 1.037x | 3.58% |
| dash_100 | 3.5749 | 3.1453 | 1.137x | 12.02% |
| dash_1000 | 95.4339 | 34.8805 | 2.736x | 63.45% |
| id_actions_1000 | 76.5883 | 28.4958 | 2.688x | 62.79% |
| sfm_group_200 | 9.4197 | 9.2325 | 1.020x | 1.99% |


The 1000-object cases show substantial improvement; 1–10-object differences and the group-SFM control are small enough to treat cautiously. This change does not optimize group SFM itself. Timings come from a Windows desktop/laptop, not an isolated benchmark host. In particular, baseline dash-1000 run medians span about 88–129 ms, so these are local observations, not universal speedup guarantees. Raw per-step samples and per-run p95 values are preserved.

The scenes use differential-drive circular robots on a 1.2 m grid, radius 0.15 m, with goals 200 m along x. Dash uses a 1 m/s speed cap; explicit-ID actions use `[0.3, 0.0]`. The two cases are compared only against their own identical baseline inputs, not against each other. Group SFM is an unchanged control. No rendering, sensor noise or LLM inference is included.

Environment: Windows 11, AMD Ryzen 9 9955HX, Python 3.13.13, NumPy 2.3.2, Shapely 2.1.2, Matplotlib 3.10.9; OpenBLAS/OMP thread limits are both 1. Source byte hashes are recorded for the measured production files; line-ending differences affect these hashes across platforms.

## Reproduction

Run from the repository root in PowerShell, with the development dependencies installed. Export the baseline into a fresh temporary directory, then run the same driver against both source trees:

```powershell
uv sync --locked --group test --group lint
if ($LASTEXITCODE -ne 0) { throw 'Dependency sync failed' }
$dispatchScratch = Join-Path $env:TEMP ('irsim-dispatch-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $dispatchScratch | Out-Null
git archive --format=tar --output="$dispatchScratch\baseline.tar" 39ed2f0e4ca006f5061d8808b19b46b2cfff2fca irsim
if ($LASTEXITCODE -ne 0) { throw 'Baseline export failed' }
tar -xf "$dispatchScratch\baseline.tar" -C $dispatchScratch
if ($LASTEXITCODE -ne 0) { throw 'Baseline extraction failed' }

.\.venv\Scripts\python.exe benchmarks/dispatch.py --source-root $dispatchScratch --label baseline --output "$dispatchScratch\baseline.json"
if ($LASTEXITCODE -ne 0) { throw 'Baseline measurement failed' }
.\.venv\Scripts\python.exe benchmarks/dispatch.py --source-root . --label optimized --output "$dispatchScratch\optimized.json"
if ($LASTEXITCODE -ne 0) { throw 'Optimized measurement failed' }
```

The driver verifies that the requested source tree supplied the imported package and sets thread limits before importing numerical libraries. For a short smoke run, use `--repeats 1 --steps 10`; use the defaults for the reported protocol. Runtime dependencies must match between versions.

## Checks and evidence

- Full pytest suite, Ruff check/format and Bandit pass locally.
- The non-blocking `ty` check reports 89 diagnostics on both baseline and optimized trees under the same configuration and dependency environment. No diagnostics were added or removed; existing type errors are not repaired by this change.
- English user-guide additions have corresponding Chinese gettext translations.
- [Baseline samples](dispatch-baseline.json), [optimized samples](dispatch-optimized.json), [validation summary](dispatch-validation.json), [Chinese report](dispatch.zh-CN.md).

Wall-clock speed is deliberately not asserted in pytest. Deterministic operation-count tests guard lookup complexity, while the benchmark reports end-to-end measurements for review.
