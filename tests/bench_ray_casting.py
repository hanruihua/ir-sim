"""Ray-casting kernel performance benchmark.

Measures latency, throughput, and memory allocation of
:func:`cast_ray_segments` (NumPy) and :func:`cast_ray_segments_numba`
(Numba JIT, when available) across a range of (N_beams x M_segments) sizes.

Usage
-----
Run and save JSON results::

    python tests/bench_ray_casting.py [--json bench_results.json] [--html bench_report.html]

Options
-------
--json PATH    Write raw results to PATH (default: bench_results.json)
--html PATH    Write the HTML performance report to PATH (default: bench_report.html)
--quick        Run fewer iterations (faster, less accurate) — useful for CI smoke checks
--no-html      Skip HTML report generation

The script can also be imported as a module; call :func:`run_benchmarks` directly.
"""

from __future__ import annotations

import argparse
import json
import time
import tracemalloc
from pathlib import Path

import numpy as np

from irsim.lib.algorithm.ray_casting_2d import SEGMENT_CHUNK_SIZE, cast_ray_segments
from irsim.lib.algorithm.ray_casting_2d_numba import (
    cast_ray_segments_numba,
    is_numba_available,
    warmup,
)

# ---------------------------------------------------------------------------
# Benchmark cases: (N_beams, M_segments)
# ---------------------------------------------------------------------------
# N_beams: typical LiDAR beam counts (16-beam RPLIDAR -> 1080-beam premium)
# M_segments: obstacle boundary segments seen per scan (sparse -> dense map)
CASES: list[tuple[int, int]] = [
    (16, 10),
    (16, 100),
    (16, 1000),
    (64, 100),
    (64, 1000),
    (64, 5000),
    (181, 100),
    (181, 1000),
    (181, 5000),
    (360, 100),
    (360, 1000),
    (360, 5000),
    (360, 10000),
    (720, 100),
    (720, 1000),
    (720, 5000),
    (1080, 100),
    (1080, 1000),
    (1080, 5000),
]


def _make_scene(n_beams: int, n_segments: int, seed: int = 42) -> tuple:
    """Generate a reproducible random scene for benchmarking."""
    rng = np.random.default_rng(seed)
    angles = np.linspace(-np.pi, np.pi, n_beams)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    origin = np.zeros(2)
    centers = rng.uniform(-10, 10, (n_segments, 2))
    ang_seg = rng.uniform(0, np.pi, n_segments)
    lengths = rng.uniform(0.1, 2.0, n_segments)
    half = lengths[:, None] * np.column_stack([np.cos(ang_seg), np.sin(ang_seg)]) / 2
    seg_start = centers - half
    seg_end = centers + half
    return origin, directions, seg_start, seg_end


def _bench_fn(
    fn,
    args: tuple,
    n_warmup: int = 3,
    n_runs: int = 20,
) -> dict:
    """Time ``fn(*args)`` and return statistics (all times in µs)."""
    for _ in range(n_warmup):
        fn(*args)
    times_us = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        fn(*args)
        times_us.append((time.perf_counter() - t0) * 1e6)
    arr = np.array(times_us)
    return {
        "mean_us": float(np.mean(arr)),
        "min_us": float(np.min(arr)),
        "p50_us": float(np.median(arr)),
        "p95_us": float(np.percentile(arr, 95)),
        "std_us": float(np.std(arr)),
    }


def _peak_memory_kb(fn, args: tuple) -> float:
    """Return the peak live-memory delta (KB) for one call to ``fn(*args)``."""
    tracemalloc.start()
    fn(*args)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return peak / 1024


def run_benchmarks(
    cases: list[tuple[int, int]] | None = None,
    n_warmup: int = 3,
    n_runs: int = 20,
    verbose: bool = True,
) -> list[dict]:
    """Run all benchmark cases and return a list of result dicts.

    Args:
        cases: List of ``(n_beams, n_segments)`` pairs.  Defaults to the
            module-level :data:`CASES`.
        n_warmup: Number of un-timed warm-up calls per case.
        n_runs: Number of timed calls per case.
        verbose: If True, print a progress table to stdout.

    Returns:
        A list of dicts, one per case, with timing and memory data.
    """
    if cases is None:
        cases = CASES

    if is_numba_available():
        if verbose:
            print("Warming up Numba JIT...", end=" ", flush=True)
        warmup()
        if verbose:
            print("done.")
    elif verbose:
        print("numba not installed — Numba column will be N/A.")

    if verbose:
        print(
            f"\n{'N':>6} {'M':>7} | {'NumPy mean µs':>14} {'p95 µs':>10} "
            f"{'peak KB':>9} {'matrix MB':>10}"
            + (
                f" | {'Numba mean µs':>14} {'speedup':>8}"
                if is_numba_available()
                else ""
            )
        )
        print("-" * (80 + (26 if is_numba_available() else 0)))

    results = []
    for n, m in cases:
        scene = _make_scene(n, m)
        args_numpy = (*scene, 15.0)

        numpy_stats = _bench_fn(cast_ray_segments, args_numpy, n_warmup, n_runs)
        peak_kb = _peak_memory_kb(cast_ray_segments, args_numpy)

        n_chunks = (m + SEGMENT_CHUNK_SIZE - 1) // SEGMENT_CHUNK_SIZE
        chunk_m = min(m, SEGMENT_CHUNK_SIZE)
        # ~4 float64 matrices of shape (chunk_m, n) per chunk pass
        matrix_mb = n_chunks * chunk_m * n * 8 * 4 / 1e6

        row: dict = {
            "n_beams": n,
            "n_segments": m,
            "n_chunks": n_chunks,
            "matrix_mb_est": matrix_mb,
            "numpy": numpy_stats,
            "peak_kb": peak_kb,
        }

        if is_numba_available():
            args_numba = (*scene, 15.0)
            numba_stats = _bench_fn(
                cast_ray_segments_numba, args_numba, n_warmup, n_runs
            )
            speedup = numpy_stats["mean_us"] / numba_stats["mean_us"]
            row["numba"] = numba_stats
            row["speedup"] = speedup

        results.append(row)

        if verbose:
            line = (
                f"{n:>6} {m:>7} | {numpy_stats['mean_us']:>14.1f}"
                f" {numpy_stats['p95_us']:>10.1f} {peak_kb:>9.0f} {matrix_mb:>10.2f}"
            )
            if is_numba_available():
                sp = row["speedup"]
                ns = row["numba"]["mean_us"]
                line += f" | {ns:>14.1f} {sp:>7.1f}x"
            print(line)

    return results


# ---------------------------------------------------------------------------
# HTML report generation
# ---------------------------------------------------------------------------


def _build_html_report(results: list[dict]) -> str:
    """Build a self-contained HTML performance report from benchmark results."""
    import json as _json

    data_json = _json.dumps(results)
    has_numba = any("numba" in r for r in results)

    speedup_section = ""
    if has_numba:
        speedup_section = """
        <div class="section">
          <h2>Numba JIT Speedup vs NumPy</h2>
          <canvas id="speedupChart" height="90"></canvas>
        </div>
        """

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Ray-Casting Benchmark</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
:root {{
  --bg: #f8f9fa; --card: #fff; --border: #dee2e6; --text: #212529;
  --muted: #6c757d; --accent: #0d6efd; --good: #198754; --warn: #dc3545;
  --font: system-ui, -apple-system, sans-serif;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --bg: #0d1117; --card: #161b22; --border: #30363d; --text: #e6edf3;
    --muted: #8b949e; --accent: #58a6ff; --good: #3fb950; --warn: #f85149;
  }}
}}
:root[data-theme="dark"] {{
  --bg: #0d1117; --card: #161b22; --border: #30363d; --text: #e6edf3;
  --muted: #8b949e; --accent: #58a6ff; --good: #3fb950; --warn: #f85149;
}}
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: var(--bg); color: var(--text); font-family: var(--font); font-size: 14px; line-height: 1.5; }}
.page {{ max-width: 1100px; margin: 0 auto; padding: 24px 20px; }}
h1 {{ font-size: 1.5rem; font-weight: 700; margin-bottom: 4px; }}
.subtitle {{ color: var(--muted); font-size: 0.875rem; margin-bottom: 28px; }}
.kpis {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; margin-bottom: 28px; }}
.kpi {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 14px 16px; }}
.kpi-label {{ font-size: 0.75rem; color: var(--muted); text-transform: uppercase; letter-spacing: .04em; margin-bottom: 4px; }}
.kpi-value {{ font-size: 1.6rem; font-weight: 700; font-variant-numeric: tabular-nums; }}
.kpi-value.good {{ color: var(--good); }}
.kpi-value.warn {{ color: var(--warn); }}
.section {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 20px; margin-bottom: 20px; overflow-x: auto; }}
.section h2 {{ font-size: 1rem; font-weight: 600; margin-bottom: 16px; }}
table {{ width: 100%; border-collapse: collapse; font-size: 0.8rem; }}
th {{ text-align: right; font-weight: 600; padding: 6px 10px; border-bottom: 2px solid var(--border); color: var(--muted); white-space: nowrap; }}
th:first-child, th:nth-child(2) {{ text-align: center; }}
td {{ text-align: right; padding: 5px 10px; border-bottom: 1px solid var(--border); font-variant-numeric: tabular-nums; white-space: nowrap; }}
td:first-child, td:nth-child(2) {{ text-align: center; font-weight: 600; }}
tr:hover td {{ background: color-mix(in srgb, var(--accent) 6%, transparent); }}
.tag {{ display: inline-block; padding: 1px 6px; border-radius: 4px; font-size: 0.7rem; font-weight: 700; }}
.tag-ok {{ background: color-mix(in srgb, var(--good) 18%, transparent); color: var(--good); }}
.tag-warn {{ background: color-mix(in srgb, var(--warn) 18%, transparent); color: var(--warn); }}
.note {{ color: var(--muted); font-size: 0.78rem; margin-top: 12px; line-height: 1.6; }}
.grid2 {{ display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }}
@media (max-width: 700px) {{ .grid2 {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<div class="page">
  <h1>IR-SIM Ray-Casting Benchmark</h1>
  <p class="subtitle">NumPy vectorised kernel · {("Numba JIT parallel kernel · " if has_numba else "numba not installed · ")}SEGMENT_CHUNK_SIZE = {SEGMENT_CHUNK_SIZE}</p>

  <div class="kpis" id="kpiRow"></div>

  <div class="section">
    <h2>Latency by Scenario (µs per scan step)</h2>
    <table id="mainTable">
      <thead>
        <tr>
          <th>N beams</th><th>M segs</th>
          <th>NumPy mean</th><th>NumPy p95</th><th>Peak KB</th><th>Matrix MB</th><th>Chunks</th>
          {"<th>Numba mean</th><th>Speedup</th>" if has_numba else ""}
          <th>Status</th>
        </tr>
      </thead>
      <tbody id="mainBody"></tbody>
    </table>
    <p class="note">
      ⚠ <strong>Status</strong>: <span class="tag tag-ok">OK</span> mean &lt; 2 ms (real-time at 10 Hz with &gt;80% headroom) ·
      <span class="tag tag-warn">SLOW</span> mean &gt; 10 ms (misses 10 Hz deadline with default dt=0.1 s)
    </p>
  </div>

  <div class="grid2">
    <div class="section">
      <h2>Throughput — NumPy (M beam·seg tests / s)</h2>
      <canvas id="throughputChart" height="200"></canvas>
    </div>
    <div class="section">
      <h2>Peak Memory Allocation (KB per scan step)</h2>
      <canvas id="memChart" height="200"></canvas>
    </div>
  </div>

  {speedup_section}

  <div class="section">
    <h2>Copy / Allocation Analysis — NumPy Kernel</h2>
    <p class="note" style="margin-bottom:12px">
      The NumPy kernel allocates <strong>4 x (chunk_size x N_beams) float64 matrices</strong>
      per chunk pass inside <code>_nonparallel_hit_distances</code>:
      <code>denominator</code>, <code>segment_position</code>, <code>ray_distance</code>,
      and <code>valid_hit</code> (bool).  With chunk_size = {SEGMENT_CHUNK_SIZE}
      and N = 360 beams, one chunk = <strong>4 x 1024 x 360 x 8 B ~ 11.8 MB</strong>.
      A 5 000-segment scene uses ⌈5000/1024⌉ = 5 chunks -> 59 MB of temporaries
      <em>per scan step</em>, dominated by memory bandwidth.
    </p>
    <table>
      <thead>
        <tr><th style="text-align:left">Step</th><th>Allocation shape</th><th>Bytes (N=360, chunk=1024)</th><th>Notes</th></tr>
      </thead>
      <tbody>
        <tr><td style="text-align:left"><code>perpendicular_directions</code></td><td>(N, 2)</td><td>5 760 B</td><td>allocated once per step</td></tr>
        <tr><td style="text-align:left"><code>denominator</code></td><td>(chunk_M, N)</td><td>2 949 120 B ~ 2.8 MB</td><td>re-allocated per chunk</td></tr>
        <tr><td style="text-align:left"><code>segment_position</code></td><td>(chunk_M, N)</td><td>2 949 120 B ~ 2.8 MB</td><td>re-allocated per chunk</td></tr>
        <tr><td style="text-align:left"><code>line_cross</code></td><td>(chunk_M,)</td><td>8 192 B</td><td>re-allocated per chunk</td></tr>
        <tr><td style="text-align:left"><code>ray_distance</code></td><td>(chunk_M, N)</td><td>2 949 120 B ~ 2.8 MB</td><td>re-allocated per chunk</td></tr>
        <tr><td style="text-align:left"><code>valid_hit</code> (bool)</td><td>(chunk_M, N)</td><td>368 640 B ~ 0.4 MB</td><td>re-allocated per chunk</td></tr>
        <tr><td style="text-align:left"><code>np.where result</code></td><td>(chunk_M, N)</td><td>2 949 120 B ~ 2.8 MB</td><td>re-allocated per chunk</td></tr>
      </tbody>
    </table>
    <p class="note" style="margin-top:12px">
      <strong>Numba JIT fix</strong>: the JIT kernel loops over beams with <code>prange</code>
      and segments with a plain <code>for</code>.  Working state per thread is 2 scalars
      (<code>best_t</code>, <code>best_j</code>) — no matrix allocation at any scale.
      Total live memory = O(N) output + O(1) per thread regardless of M.
    </p>
  </div>

  <div class="section">
    <h2>Optimisation Roadmap</h2>
    <table>
      <thead><tr><th style="text-align:left">Option</th><th style="text-align:left">Memory</th><th style="text-align:left">Speedup (est.)</th><th style="text-align:left">Notes</th></tr></thead>
      <tbody>
        <tr>
          <td style="text-align:left"><strong>NumPy baseline</strong> (current)</td>
          <td style="text-align:left">O(chunk x N) per chunk</td>
          <td style="text-align:left">1x (reference)</td>
          <td style="text-align:left">Single-threaded, bandwidth-bound for large M</td>
        </tr>
        <tr>
          <td style="text-align:left"><strong>Numba JIT</strong> (<code>parallel=True</code>, <code>cache=True</code>)</td>
          <td style="text-align:left">O(N) — no large temporaries</td>
          <td style="text-align:left">10-50x (all cores)</td>
          <td style="text-align:left"><code>pip install numba</code>; warmup once; zero build step</td>
        </tr>
        <tr>
          <td style="text-align:left"><strong>C++ / OpenMP</strong></td>
          <td style="text-align:left">O(N) — registers only</td>
          <td style="text-align:left">15-60x (vectorised + all cores)</td>
          <td style="text-align:left">Needs pybind11 + C compiler; identical algorithm; see module docstring</td>
        </tr>
        <tr>
          <td style="text-align:left"><strong>C++ / TBB</strong></td>
          <td style="text-align:left">O(N)</td>
          <td style="text-align:left">20-70x (work-stealing scheduler)</td>
          <td style="text-align:left">Intel TBB; dynamic load balancing over beams; best for uneven segment distribution</td>
        </tr>
        <tr>
          <td style="text-align:left"><strong>angle_std beam jitter</strong> (fix)</td>
          <td style="text-align:left">O(N) extra for perturbed dirs</td>
          <td style="text-align:left">~-1% (tiny overhead)</td>
          <td style="text-align:left">Currently a no-op bug; add <code>rng.normal(0, angle_std, N)</code> to angle_list before direction build</td>
        </tr>
        <tr>
          <td style="text-align:left"><strong>range_min enforcement</strong> (fix)</td>
          <td style="text-align:left">N/A</td>
          <td style="text-align:left">no change</td>
          <td style="text-align:left">Add <code>ORIGIN_EPS = max(ORIGIN_EPS, range_min)</code> in scan init</td>
        </tr>
      </tbody>
    </table>
  </div>
</div>

<script>
const data = {data_json};
const hasNumba = {"true" if has_numba else "false"};

// KPI row
const kpiData = [
  {{label:'Scenarios Tested', value: data.length, cls:''}},
  {{label:'Max NumPy Latency', value: Math.round(Math.max(...data.map(r=>r.numpy.mean_us)))+' µs', cls:'warn'}},
  {{label:'Min NumPy Latency', value: Math.round(Math.min(...data.map(r=>r.numpy.mean_us)))+' µs', cls:'good'}},
  {{label:'Peak Memory', value: Math.round(Math.max(...data.map(r=>r.peak_kb)))+' KB', cls:'warn'}},
];
if(hasNumba) {{
  const speedups = data.filter(r=>r.speedup).map(r=>r.speedup);
  kpiData.push({{label:'Max Speedup', value: Math.round(Math.max(...speedups))+'x', cls:'good'}});
  kpiData.push({{label:'Min Speedup', value: Math.round(Math.min(...speedups))+'x', cls:''}});
}}
const kpiRow = document.getElementById('kpiRow');
kpiData.forEach(k => {{
  kpiRow.innerHTML += `<div class="kpi"><div class="kpi-label">${{k.label}}</div><div class="kpi-value ${{k.cls}}">${{k.value}}</div></div>`;
}});

// Main table
const tbody = document.getElementById('mainBody');
data.forEach(r => {{
  const slow = r.numpy.mean_us > 10000;
  const tag = slow ? '<span class="tag tag-warn">SLOW</span>' : '<span class="tag tag-ok">OK</span>';
  const numbaCol = hasNumba
    ? `<td>${{r.numba ? r.numba.mean_us.toFixed(1) : 'N/A'}}</td><td>${{r.speedup ? r.speedup.toFixed(1)+'x' : 'N/A'}}</td>`
    : '';
  tbody.innerHTML += `<tr>
    <td>${{r.n_beams}}</td><td>${{r.n_segments}}</td>
    <td>${{r.numpy.mean_us.toFixed(1)}}</td><td>${{r.numpy.p95_us.toFixed(1)}}</td>
    <td>${{r.peak_kb.toFixed(0)}}</td><td>${{r.matrix_mb_est.toFixed(2)}}</td><td>${{r.n_chunks}}</td>
    ${{numbaCol}}<td>${{tag}}</td>
  </tr>`;
}});

// Chart helper
const isDark = ()=> document.documentElement.dataset.theme === 'dark' ||
  (!document.documentElement.dataset.theme && window.matchMedia('(prefers-color-scheme:dark)').matches);
const gridColor = ()=> isDark() ? 'rgba(255,255,255,0.08)' : 'rgba(0,0,0,0.07)';
const textColor = ()=> isDark() ? '#8b949e' : '#6c757d';

// Throughput chart
const labels = data.map(r=>`${{r.n_beams}}x${{r.n_segments}}`);
new Chart(document.getElementById('throughputChart'), {{
  type: 'bar',
  data: {{
    labels,
    datasets: [{{
      label: 'NumPy (M ops/s)',
      data: data.map(r => (r.n_beams * r.n_segments / r.numpy.mean_us).toFixed(2)),
      backgroundColor: 'rgba(13,110,253,0.7)',
      borderRadius: 3,
    }}]
  }},
  options: {{
    responsive:true, maintainAspectRatio:true,
    plugins:{{legend:{{display:false}}}},
    scales:{{
      x:{{ticks:{{font:{{size:9}},color:textColor(),maxRotation:60}},grid:{{color:gridColor()}}}},
      y:{{ticks:{{color:textColor()}},grid:{{color:gridColor()}},title:{{display:true,text:'M tests/s',color:textColor()}}}}
    }}
  }}
}});

// Memory chart
new Chart(document.getElementById('memChart'), {{
  type: 'bar',
  data: {{
    labels,
    datasets: [{{
      label: 'Peak KB',
      data: data.map(r => r.peak_kb.toFixed(1)),
      backgroundColor: 'rgba(220,53,69,0.7)',
      borderRadius: 3,
    }}]
  }},
  options: {{
    responsive:true, maintainAspectRatio:true,
    plugins:{{legend:{{display:false}}}},
    scales:{{
      x:{{ticks:{{font:{{size:9}},color:textColor(),maxRotation:60}},grid:{{color:gridColor()}}}},
      y:{{ticks:{{color:textColor()}},grid:{{color:gridColor()}},title:{{display:true,text:'KB',color:textColor()}}}}
    }}
  }}
}});

// Speedup chart
if(hasNumba) {{
  const numbaData = data.filter(r=>r.speedup);
  new Chart(document.getElementById('speedupChart'), {{
    type: 'bar',
    data: {{
      labels: numbaData.map(r=>`${{r.n_beams}}x${{r.n_segments}}`),
      datasets: [{{
        label: 'Speedup (x)',
        data: numbaData.map(r=>r.speedup.toFixed(1)),
        backgroundColor: numbaData.map(r=> r.speedup > 10 ? 'rgba(25,135,84,0.8)' : 'rgba(255,193,7,0.8)'),
        borderRadius: 3,
      }}]
    }},
    options: {{
      responsive:true, maintainAspectRatio:true,
      plugins:{{legend:{{display:false}}}},
      scales:{{
        x:{{ticks:{{font:{{size:9}},color:textColor(),maxRotation:60}},grid:{{color:gridColor()}}}},
        y:{{ticks:{{color:textColor()}},grid:{{color:gridColor()}},title:{{display:true,text:'Speedup x',color:textColor()}}}}
      }}
    }}
  }});
}}
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--json", default="bench_results.json", metavar="PATH", help="Output JSON path"
    )
    p.add_argument(
        "--html",
        default="bench_report.html",
        metavar="PATH",
        help="Output HTML report path",
    )
    p.add_argument("--no-html", action="store_true", help="Skip HTML generation")
    p.add_argument(
        "--quick", action="store_true", help="Fewer iterations (CI smoke check)"
    )
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    n_warmup = 1 if args.quick else 3
    n_runs = 5 if args.quick else 20
    cases = CASES[:6] if args.quick else CASES

    print(
        f"IR-SIM ray-casting benchmark  [numba={'yes' if is_numba_available() else 'no'}]"
    )
    results = run_benchmarks(
        cases=cases, n_warmup=n_warmup, n_runs=n_runs, verbose=True
    )

    json_path = Path(args.json)
    json_path.write_text(json.dumps(results, indent=2))
    print(f"\nJSON saved -> {json_path}")

    if not args.no_html:
        html = _build_html_report(results)
        html_path = Path(args.html)
        html_path.write_text(html, encoding="utf-8")
        print(f"HTML report -> {html_path}")

    # Print GitHub Actions step summary if running in CI
    if "GITHUB_STEP_SUMMARY" in __import__("os").environ:
        summary_path = __import__("os").environ["GITHUB_STEP_SUMMARY"]
        lines = [
            "## Ray-Casting Benchmark Summary\n",
            f"| N beams | M segs | NumPy mean µs | Peak KB | {'Speedup |' if any('numba' in r for r in results) else ''}",
            f"|---------|--------|---------------|---------|{'---------|' if any('numba' in r for r in results) else ''}",
        ]
        for r in results:
            sp_col = f" {r['speedup']:.1f}x |" if "speedup" in r else ""
            lines.append(
                f"| {r['n_beams']} | {r['n_segments']} | {r['numpy']['mean_us']:.1f} | {r['peak_kb']:.0f} |{sp_col}"
            )
        with open(summary_path, "a") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
