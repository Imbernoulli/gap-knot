# fleet: gap-knot rollouts on Daytona CPU sandboxes

One knot-tying episode = one process = one sandbox at a time. `fleet/` builds the
environment once as a Daytona snapshot, keeps a pool of CPU sandboxes made from it, and
runs jobs (shell commands under the RSS venv, with a fresh tarball of this checkout) across
the pool, bringing back each job's outputs as a tarball.

Run everything with the interpreter that has the `daytona` SDK (0.207):
`/srv/home/bohanlyu/miniconda3/bin/python3` (the RSS venv does not have it).

| file | what |
|---|---|
| `snapshot.py` | the image recipe and `gap-knot-rss-<hash>` snapshot build |
| `pool.py` | `Pool` / `Job` / `JobResult`: up, down, status, run |
| `cli.py` | the same from a shell |
| `common.py` | key loading, ownership labels, ledger, prices |
| `remote/sitecustomize.py` | in every job: SuperDex worker threads = the sandbox's vCPUs |
| `bench/knot_step.py`, `bench/run_bench.py` | the step-time benchmark below |
| `state/` (gitignored) | `ledger.jsonl` (every sandbox id created/deleted), snapshot build logs, results, bench output |

Safety: every sandbox carries `owner=bohan, label=bohan, purpose=gap-knot` (+ `gk-pool=<name>`),
is `ephemeral` with `auto_stop_interval=45` min (an idle sandbox stops and is deleted), and
nothing in `fleet/` lists, stops or deletes a sandbox without `owner=bohan` and
`purpose=gap-knot`. At most 24 such sandboxes are alive at once (`common.MAX_ALIVE`). The
API key is read from `mooncake/runtime/secrets/daytona_api_key_ondemand` into the process
environment only; it is never logged or sent into a sandbox.

## 1. The snapshot

```bash
python fleet/cli.py snapshot build          # builds (or finds) gap-knot-rss-<hash>, 2 vCPU / 4 GiB / 30 GiB
python fleet/cli.py snapshot dockerfile     # the recipe
python fleet/cli.py snapshot list
```

Current: **`gap-knot-rss-abed33ba`** (9.7 GB). Contents (`/opt/gk`):

* `robosimstudio/` at b001cc7e (THIRD_PARTY.lock), `.venv` from
  `UV_NO_GROUP=rtx uv sync --frozen --extra cpu --extra gap --extra superdex` on Python 3.12.13,
  uv 0.11.14. For a machine without a GPU `examples/install.md` names `--extra cpu` (= `robosimstudio[superdex]`)
  with the `rtx` group off; `--extra superdex` is then redundant but harmless, `--extra gap` brings the graph
  runtime. `--frozen` installs uv.lock as is, and the lock pins torch's CUDA build, so about 6 GB of NVIDIA
  wheels (torch's and cuRobo's) come along unused; dropping them would mean leaving the lock.
* `graph-as-policy/` at 71549e15 and `open-robot-skills/` at 752dfbd9 beside it (RSS's
  `[tool.gap] registries = ["../open-robot-skills"]` resolves).
* Warp's CPU kernel cache and RSS's convex-piece cache, warmed by stepping the knot scene during the
  build. Scene build in a fresh sandbox: 5-7 s from this snapshot, against 23-32 s from the earlier
  unwarmed one (d3449755, since deleted), which compiled them in its first episode.
* `jobs/` (per-job dirs) and, per job, `gap-knot/` (the code bundle).

The hash covers THIRD_PARTY.lock and the Dockerfile text, so bumping a pin builds a new
snapshot. A full build takes 9-12 min (Daytona's builder did not reuse layers between
builds). A snapshot fixes the sandbox shape: Daytona refuses
resources alongside a snapshot (`400 Cannot specify Sandbox resources when using a snapshot`)
and this API has no resize (`404 Cannot POST /api/sandbox/<id>/resize`). So every other shape
is its own snapshot, `gap-knot-rss-<hash>-c<cpu>m<mem>d<disk>`, which `Pool` creates on first
use. Daytona reuses the build for the identical Dockerfile, so that takes under a second. Snapshots
count against the org's quota of 20 (2 belonged to others when this was set up), so delete
the shapes you stop using: `python fleet/cli.py snapshot delete --name ...` (it refuses names outside
`gap-knot-rss-*`).

Verified in a sandbox from the snapshot (2 vCPU / 4 GiB): `python -m unittest
tests.test_knot_pull.TestBothHandsPullTheKnotTight -v` passes 5 of 6. The one that fails is the
90 ms timing assertion, at 93.0 and 102.9 ms a step in two runs. With SuperDex left at its
default thread count it fails at 296.6 ms. On a 4 vCPU sandbox with 4 threads all 6 pass.

## 2. Pool: up / run / down

```bash
python fleet/cli.py up 8                         # pool "default", 2 vCPU / 4 GiB; parallel create + health check
python fleet/cli.py status [--all] [--json]      # labelled listing, $/h
python fleet/cli.py run jobs.json --keep         # reuse the pool afterwards
python fleet/cli.py run jobs.json --size 24 --stream --results runs/it7
python fleet/cli.py down [--pool NAME | --all]   # deletes, then re-lists to verify nothing is left
python fleet/cli.py ledger --tail 20
```

`--pool/--cpu/--memory/--disk/--snapshot/--auto-stop` choose the pool and shape. `run`
deletes the pool's sandboxes when it finishes unless `--keep`. While it runs, each sandbox
is deleted as soon as no jobs are left to start.

`jobs.json` is a list of jobs:

```json
[{"id": "ep-0001",
  "cmd": "python -m knotlab.episode --seed 1 --out out",
  "files": {"/abs/local/graph.json": "in/graph.json"},
  "outputs": ["out"],
  "timeout_s": 3600,
  "env": {"SEED": "1"},
  "cwd": null}]
```

What a job sees:

* `cwd` defaults to `/opt/gk/gap-knot`, the bundle: this checkout minus `third_party/`, `runs/`, `video/`,
  `.git`, `fleet/state`, `__pycache__`.
* `PATH`/`VIRTUAL_ENV` are the RSS venv.
* `PYTHONPATH=/opt/gk/gap-knot:/opt/gk/gap-knot/fleet/remote:/opt/gk/robosimstudio`.
* `GK_JOB_ID` and `GK_JOB_DIR` (`/opt/gk/jobs/<id>.<attempt>`) are set.
* `GK_CPUS`, `OMP_NUM_THREADS` etc. and `GK_SUPERDEX_THREADS` are set to the sandbox's vCPUs.
* A relative remote path in `files` lands under `$GK_JOB_DIR`. A relative path in `outputs` is
  taken from `cwd`.
* stdout and stderr stream into `<results>/<id>/stdout.log` and `<id>/stderr.log` while the job runs.

At the end the outputs and logs (`stdout.log`, `stderr.log`, `run.sh`, `exit_code`, `start`/`end`,
`memory_peak`) are packed as `<results>/<id>/outputs.tar.gz` and extracted to `<results>/<id>/outputs/`.

Each job writes one line to `<results>/results.jsonl` (`JobResult`):

* `status`: one of `ok`, `failed` (non-zero exit), `timeout`, `lost`, `error`.
* `exit_code`, `attempts`, and `sandboxes` (one id per attempt).
* `run_s` (the command alone, timed in the sandbox) and `wall_s` (dispatch to result).
* `missing_outputs`, the stdout/stderr tails, and `error`/`errors`.

A sandbox that disappears or stops answering, or a job past `timeout_s`, is retried once: the
sandbox is deleted (a timed-out job's whole process group is killed first) and a fresh one is
created. A job that exits non-zero is not retried.

From Python (the self-learning loop):

```python
import sys; sys.path.insert(0, "/srv/home/bohanlyu/gap-knot")
from fleet.pool import Pool, Job

pool = Pool("loop")                  # defaults: cpu=2, memory=4, disk=30 -> gap-knot-rss-abed33ba
pool.up(24)                          # idempotent: tops the pool up to 24 started sandboxes
for it in range(n_iters):
    jobs = [Job(id=f"it{it}-ep{k}", cmd=f"python -m knotlab.episode --seed {k} --out out",
                outputs=["out"], timeout_s=3600, env={"ITER": str(it)}) for k in range(48)]
    results = pool.run(jobs, results_dir=f"runs/it{it}")   # keep=True by default; re-bundles the code
    ok = [r for r in results if r.status == "ok"]          # r.outputs_dir / "out" holds each episode
pool.down()                          # or: with Pool("loop") as pool: ...  (down on exit)
```

`Pool.run` re-tars the checkout on every call and uploads it only to sandboxes whose bundle
hash differs. Sandboxes idle longer than `auto_stop` (45 min) between iterations stop and
are deleted by Daytona. The next `run` just creates new ones, in 1-2 s each.

## 3. Benchmark: knot-pull control step vs sandbox shape

How it was measured:

* `bench/knot_step.py` drives exactly `tests/test_knot_pull.py`'s `_drive`: both hands close on the tails
  and pull the overhand knot tight, about 1142 control steps.
* It records the test's own number, `tight` (mean of the last 60 steps, knot pulled tight), and `mean`
  (every step of the drive).
* `--spacing 0.004` resamples the same knot to 176 nodes, about the 180-node 1.5 m rope of a real episode.
* All runs were on AMD EPYC-Milan runners (48 logical CPUs; the 32/64 vCPU re-run landed on 256-CPU hosts),
  with SuperDex threads = vCPUs unless stated.
* Episode = 3500 steps at the mean step plus ~25 s build.
* $ = Daytona list price: $0.0504/vCPU-h, $0.0162/GiB-h RAM, $0.000108/GiB-h disk beyond 5 GiB
  (daytona.io/pricing, billed per second; the same rates as `mooncake`'s fleet).

**Target rope (176 nodes), 2 runs per row, ms per step (tight / mean):**

| vCPU / RAM | $/h | tight | mean | episode | $/episode | episodes/h/$ |
|---|---|---|---|---|---|---|
| 1 / 4 GiB | 0.118 | 519 / - | 255 / - (2nd run killed after 24 min: runner at load 58/48) | 15.5 min | 0.030 | 33 |
| **2 / 4 GiB** | **0.168** | 298 / 307 | 155 / 158 | **9.6 min** | **0.027** | **37** |
| 2 / 2 GiB (3 runs) | 0.136 | 310 / 314 / 490 | 168 / 159 / 282 | 10.0 min (typ.) | 0.023 | 44 |
| 4 / 4 GiB | 0.269 | 245 / 219 | 129 / 117 | 7.7 min | 0.034 | 29 |
| 8 / 4 GiB | 0.471 | 168 / 208 | 94 / 111 | 6.5 min | 0.051 | 20 |
| 8 / 4 GiB, 4 threads | 0.471 | 227 / 249 | 112 / 129 | 7.5 min | 0.059 | 17 |
| 16 / 4 GiB | 0.874 | 175 / 174 | 95 / 92 | 5.9 min | 0.086 | 12 |
| 16 / 4 GiB, 8 threads | 0.874 | 165 / 169 | 97 / 94 | 6.2 min | 0.089 | 11 |

A third 2 vCPU / 4 GiB run, in the final check on a fresh sandbox, gave 397 / 185 ms (see the noise note below).

**The test's scene (88 nodes), one run each unless noted (tight / mean ms):**

| vCPU | threads = vCPUs | other thread counts | SuperDex default (-1 → 22 workers) |
|---|---|---|---|
| 1 | 119 / 87 | 0 (single-threaded): 136 / 93 | 645 / 442 |
| 2 | 78 / 61; again 100 / 69, 84 / 66, 105 / 73, 103 / 80 | 1: 175 / 129 | 297 / 213 |
| 4 | 65 / 53 | 2: 132 / 105 | 147 / 107 |
| 8 | 59 / 45 | 4: 73 / 56 | 69 / 50 |
| 16 | 107 / 85 | 4: 69 / 51; 8: 105 / 86 | |
| 32 | 130 / 69 | 8: 89 / 55; 16: 428 / 488 (first, contended run) | |
| 64 | 141 / 72 | 16: 132 / 63; 8: 500 / 471 (first, contended run) | |

For comparison, the local dev box (EPYC 9654, 32 pinned cores, 32 threads): 127 / 81 ms; 176
nodes on 4 pinned cores, 4 threads: 216 / 107 ms. Peak memory (cgroup, page cache included)
was 0.9-1.3 GiB in every run, 2 GiB shapes included.

What the numbers say:

* **SuperDex must be told the quota.** Inside a sandbox `nproc`/`os.cpu_count()` report the host's 48
  (or 256) CPUs and the quota is only in `/sys/fs/cgroup/cpu.max`. RSS's default `threads=-1` then starts
  ~22 workers on 1-4 vCPUs, and steps run 2-5x slower. `remote/sitecustomize.py` turns that -1 into
  `GK_SUPERDEX_THREADS` (= vCPUs) in every job. An explicit `ensure_initialized(n)` is left alone, and
  `env={"GK_SUPERDEX_THREADS": "-1"}` opts a job out.
* **The rope stops scaling at about 8 threads.** 16, 32 and 64 vCPU are no faster than 8, and cost 2-7x
  more per episode.
* **Runners are shared, so timings are noisy.** Identical 2 vCPU runs ranged 155-282 ms mean.
  1 vCPU is fragile: one of two 176-node runs took over 23 min instead of 5.5, CPU-throttled 75% of
  periods with 228 threads on one vCPU. Bigger shapes can also land on runners that have not cached the
  9.7 GB snapshot: the first 32/64 vCPU sandboxes took 300 s to start, and stepped at 400-500 ms while
  six of them pulled and ran together.

**Recommendation:**

* **2 vCPU / 4 GiB** (the default, threads = 2) is cheapest per episode with a usable margin: ~$0.027 and
  ~10 min per 3500-step episode on the target rope; 24 sandboxes ≈ 150 episodes/h for **$4.0/h**.
* 2 GiB instead of 4 saves another 19% ($0.023/episode). Every run here peaked at about 1 GiB, but a
  graph runtime that loads torch, or recording video, will need more; measure first.
* If the 24-sandbox cap binds and wall time matters more than dollars:
  * 4 vCPU: ≈ 190 episodes/h at $6.5/h, +25% $/episode.
  * 8 vCPU: ≈ 220 episodes/h at $11.3/h, +90% $/episode.
* Avoid 1 vCPU (fragile) and anything past 8 threads.

Cost per episode-hour, i.e. one sandbox busy with episodes for an hour:

| sandbox | per hour |
|---|---|
| 2 vCPU / 4 GiB | $0.168 (~6.2 episodes) |
| 4 vCPU / 4 GiB | $0.269 (~7.8 episodes) |
| 8 vCPU / 4 GiB | $0.471 (~9.2 episodes) |

To reproduce (pools are deleted when done):

```bash
python fleet/bench/run_bench.py --grid "2:2;4:4;8:8,4;16:16,8" --repeats 2 --spacing 0.004 --memory 4
```

## Daytona limits and quirks seen

* **Per sandbox** (org quota, region `us`): up to 64 vCPU, 128 GiB RAM and 500 GiB disk for an
  *ephemeral* sandbox. `max_disk_per_non_ephemeral_sandbox` is 0, which is why non-ephemeral creates fail.
* **Org totals**: 20000 vCPU / 80000 GiB RAM / 200000 GiB disk (about 2900 vCPU in use by others at the
  time) and 20 snapshots. The ondemand key gets 401 on the organization endpoints but can read
  `get_organization_usage_overview`.
* **Runner size**: runners have 48 logical CPUs (some 256), so a 64 vCPU sandbox on a 48-CPU runner cannot
  get 64.
* **Startup**: creating from the snapshot takes 1-2 s plus a ~1 s health check when the runner has the
  image, and ~300 s when it has to pull it.
* **Label listing lags**: it misses a just-created sandbox, and still shows a just-deleted one, for a few
  seconds. `Pool` tracks the ids it created, and `down` re-lists after a pause.
* **Activity**: auto-stop counts toolbox activity. The runner polls each job every 5 s and calls
  `refresh_activity` every 5 min, so a long episode is not stopped under it.
* **`tests` name clash**: gap-knot has its own `tests` package, and it shadows RSS's on the job PYTHONPATH.
  Run RSS tests as `python -m unittest tests....` with `cwd=/opt/gk/robosimstudio`, where the cwd comes
  first. `bench/knot_step.py` puts the RSS root first itself.
