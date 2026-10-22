# RSS resource benchmark, 2026-10-22

One simultaneous pair of short `grasp_probe` episodes used the exact same frozen graph
and nominal overhand variant (seed 0), with a 1200-step cap. This is a performance
probe, not a knot-tying demonstration. Both episodes exited normally and saved finite
96×3 final rope geometry. Runtime knot success is false for both.

| Shape | Job execution | Complete lifecycle | Simulated time |
|---|---:|---:|---:|
| 2 vCPU / 4 GiB | 71.31 s | 92.76 s | 8.77 s |
| 8 vCPU / 16 GiB | 59.17 s | 76.15 s | 8.75 s |

The 8-vCPU shape was successfully created from the project RSS snapshot and passed
its RSS/SuperDex health check. Job execution was 1.205× faster in this single pair
(17% shorter). RSS startup and IK are included; this does not establish sustained
throughput, cost efficiency, or a speedup for complete knot-tying graphs. Different
thread counts produced a small difference in simulated duration. Repeat measurements
are needed before interpreting small timing changes.

The two unique project-owned pools were deleted and each reported zero remaining
sandboxes. No other pool was modified. The earlier run in `runs/rss_cpu_benchmark_20261022`
is superseded because its concurrent artifact downloads shared a directory. These
numbers use only the corrected, isolated v2 run.

`manifest.json` records the frozen graph hash, inputs, grader, and run parameters;
`summary.json` records shapes, job execution times, and pool lifecycle measurements;
`cleanup.json` records deletion verification. Each shape directory preserves the
result, exact final rope geometry, and audit. `frozen_graph/` is the common graph.
The unique pool identifiers are resource bookkeeping, not account credentials.
