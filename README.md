# MRCPP — Multi-Robot Coverage Path Planner

## Overview

This project provides three executables:

| Binary | Purpose |
|---|---|
| `mrcpp_basic` | Plan coverage paths for one experiment and save per-robot GPS paths |
| `mrcpp_ablation` | Run a full three-study ablation sweep over one experiment |
| `energy_estimator` | Compute energy consumption from saved path CSV files |

---

## 1. Build

```bash
mkdir -p build && cd build
cmake ..
make mrcpp_basic mrcpp_ablation energy_estimator
cd ..
```

Binaries are placed in `build/`.

---

## 2. Directory Layout

Before running any tool, copy `experiments/` and `config/` into `results/`.
This keeps raw inputs separate from generated outputs.

```
results/
  experiments/
    <env_name>/
      request.json        # problem definition
      start_point.csv     # lat, lon of the depot/start point
      mrcpp_basic.xml     # behavior tree for mrcpp_basic
  config/
    mrcpp_bt_ablation.xml              # ablation BT (GPS)
    mrcpp_bt_ablation_cartesian.xml    # ablation BT (Cartesian)
```

The helper script `analysis/setup_results.sh` does this automatically (see §5).

---

## 3. `mrcpp_basic`

Runs the coverage path planner on a single experiment and writes one CSV per robot.

### Required files inside `<experiment_dir>`

| File | Description |
|---|---|
| `request.json` | Problem definition (polygon, obstacles, num\_robots, …) |
| `start_point.csv` | Single line: `lat, lon` |
| `mrcpp_basic.xml` | Behavior tree XML |

### Usage

```bash
./build/mrcpp_basic <experiment_dir>
```

### Example

```bash
./build/mrcpp_basic results/experiments/simple
```

### Output

Path CSVs are written to `<experiment_dir>/mrcpp_results/`:

```
results/experiments/simple_6/mrcpp_results/
  path_1.csv
  path_2.csv
  path_3.csv
```

Each CSV has columns `Latitude,Longitude` and includes the depot as first and last waypoint.

### Run all experiments

```bash
for d in results/experiments/*/; do
    ./build/mrcpp_basic "$d"
done
```

---

## 4. `mrcpp_ablation`

Runs three ablation studies over a single experiment:

| Study | What varies | Fixed values |
|---|---|---|
| 1 — Orientation | `mar`, `angle_search`, `pca`, `min_width` | headland\_scale=1.0, transition=full |
| 2 — Headland buffer | 0.0, 0.25, 0.5, 0.75, 1.0 | orientation=mar, transition=full |
| 3 — Transition strategy | `full`, `direct`, `dijkstra_only` | orientation=mar, headland\_scale=2.0 |

### Required files inside `<experiment_dir>`

| File | Description |
|---|---|
| `request.json` | Problem definition |
| `start_point.csv` | Single line: `lat, lon` |

The binary also needs `config/mrcpp_bt_ablation.xml` (and optionally
`config/mrcpp_bt_ablation_cartesian.xml`). It searches upward from the
experiment directory until it finds the `config/` folder automatically.

### Usage

```bash
./build/mrcpp_ablation <experiment_dir>
```

### Example

```bash
./build/mrcpp_ablation results/experiments/simple_6
```

### Output

Three CSVs are written to `<experiment_dir>/mrcpp_ablation/`:

```
results/experiments/simple_6/mrcpp_ablation/
  ablation_orientation.csv
  ablation_headland.csv
  ablation_transition.csv
```

CSV columns: `study, data_file, orientation, headland_scale, transition,`
`num_robots, Et_Wh, Et_bar_Wh, total_path_len_m, max_path_len_m,`
`total_waypoints, compute_time_ms, success`

### Run all experiments + merge

Use the provided script, which runs every experiment and merges per-study results
into a single `ablation_*_all.csv` in `results/experiments/`:

```bash
bash analysis/run_ablation.sh
# or with explicit paths:
bash analysis/run_ablation.sh results/experiments build/mrcpp_ablation
```

Merged outputs:

```
results/experiments/
  ablation_orientation_all.csv
  ablation_headland_all.csv
  ablation_transition_all.csv
```

---

## 5. `energy_estimator`

Computes energy consumption from a directory of path CSVs (e.g. the output of
`mrcpp_basic`).

### Usage

```bash
./build/energy_estimator <directory_with_csv_paths> [--latlon]
```

| Flag | Description |
|---|---|
| *(none)* | Treats CSV columns as `X, Y` in **metres** |
| `--latlon` | Treats CSV columns as `latitude, longitude` in **degrees** and converts to metres automatically |

### Example — use output from `mrcpp_basic`

```bash
./build/energy_estimator results/experiments/simple/mrcpp_results --latlon
```

### Example — Cartesian paths

```bash
./build/energy_estimator results/experiments/rect/mrcpp_results
```

### Output (stdout)

```
File                         Len[m]     Energy[J]   Energy[Wh]   kJ/km
---------------------------------------------------------------
path_1.csv                  1254.30      5432.10       1.51      4.33
path_2.csv                  1341.80      5821.50       1.62      4.34
---------------------------------------------------------------
path_cost_sum (fleet energy): 11253.60 J  (3.13 Wh)
max_path_cost  (worst UAV):    5821.50 J  (1.62 Wh)  in path_2.csv
TOTAL length: 2596.10 m
```

---

## 6. `main.py` — Batch energy analysis tool

Python script to analyze energy consumption for multiple experiments and planning methods stored in `planner_data/`.

### Purpose

Processes path data from various planning algorithms (EAMCMP, MRCPP, POPCORN, DARP) across different experimental scenarios and computes energy estimates using the `energy_estimator` binary.

### Usage

```bash
python main.py
```

### Configuration

Edit the `main()` function to select:

- **Benchmarks**: List of environment names (e.g., `complex_4`, `complex_5`, `cape_4`, etc.)
- **Methods**: Planning algorithms to analyze (e.g., `["eamcmp", "mrcpp", "popcorn", "darp"]`)
- **Data directory**: Defaults to `planner_data/scale_result_with_start_end/`

### What it does

1. Iterates through each benchmark environment and method combination
2. Locates CSV path files in `planner_data/scale_result_with_start_end/{env_name}/{method}/`
3. Calls `energy_estimator` on each valid directory containing CSV files
4. Saves energy estimation output to `energy_estimation_with_initial_location.txt` in each method directory

### Optional: Add start/end depot

The `estimate_energy()` method supports an `append_initial_location` flag that:
- Reads the depot location from `start_point.csv`
- Prepends and appends this location to each robot's path CSV
- Ensures energy estimates include depot return trips

### Example output

```
Processing complex_4/eamcmp...
Processing complex_4/mrcpp...
Processing complex_5/eamcmp...
...
```

Energy results are saved to:
```
planner_data/scale_result_with_start_end/complex_4/eamcmp/energy_estimation_with_initial_location.txt
planner_data/scale_result_with_start_end/complex_4/mrcpp/energy_estimation_with_initial_location.txt
...
```

---

## 7. Setup & automation script

The script `analysis/setup_results.sh` bootstraps a clean `results/` workspace
and can optionally run all three tools in sequence.

```bash
bash analysis/setup_results.sh
```

See §8 for options.

---

## 8. Full workflow (end-to-end)

```bash
# 1. Build
mkdir -p build && cd build && cmake .. && make mrcpp_basic mrcpp_ablation energy_estimator && cd ..

# 2. Prepare results directory
bash analysis/setup_results.sh

# 3. Run basic planner on all experiments
for d in results/experiments/*/; do
    ./build/mrcpp_basic "$d"
done

# 4. Run ablation studies + merge CSVs
bash analysis/run_ablation.sh

# 5. Estimate energy for each experiment
for d in results/experiments/*/mrcpp_results/; do
    echo "=== $d ==="
    ./build/energy_estimator "$d" --latlon
done
```
