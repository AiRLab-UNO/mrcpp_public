#!/usr/bin/env bash
# setup_results.sh — Bootstrap the results workspace and optionally run all tools.
#
# Usage:
#   bash analysis/setup_results.sh [OPTIONS]
#
# Options:
#   --run-basic       Run mrcpp_basic on every experiment after setup
#   --run-ablation    Run mrcpp_ablation on every experiment after setup
#   --run-energy      Run energy_estimator on every mrcpp_results dir after setup
#   --run-all         Equivalent to --run-basic --run-ablation --run-energy
#   --results-dir D   Override the results directory (default: <project_root>/results)
#   --build-dir D     Override the build directory   (default: <project_root>/build)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── defaults ─────────────────────────────────────────────────────────────────
RESULTS_DIR="$PROJECT_ROOT/results"
BUILD_DIR="$PROJECT_ROOT/build"
RUN_BASIC=0
RUN_ABLATION=0
RUN_ENERGY=0

# ── argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-basic)     RUN_BASIC=1    ;;
        --run-ablation)  RUN_ABLATION=1 ;;
        --run-energy)    RUN_ENERGY=1   ;;
        --run-all)       RUN_BASIC=1; RUN_ABLATION=1; RUN_ENERGY=1 ;;
        --results-dir)   RESULTS_DIR="$2"; shift ;;
        --build-dir)     BUILD_DIR="$2";   shift ;;
        *) echo "[setup] Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

RESULTS_EXPERIMENTS="$RESULTS_DIR/experiments"
RESULTS_CONFIG="$RESULTS_DIR/config"
BASIC_BIN="$BUILD_DIR/mrcpp_basic"
ABLAT_BIN="$BUILD_DIR/mrcpp_ablation"
ENERGY_BIN="$BUILD_DIR/energy_estimator"

log() { echo "[setup_results] $*"; }

# ── helper functions ──────────────────────────────────────────────────────────

check_build() {
    local missing=()
    [[ -x "$BASIC_BIN"  ]] || missing+=("mrcpp_basic")
    [[ -x "$ABLAT_BIN"  ]] || missing+=("mrcpp_ablation")
    [[ -x "$ENERGY_BIN" ]] || missing+=("energy_estimator")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log "Warning: binary/binaries not found in $BUILD_DIR: ${missing[*]}"
        log "  Build with: cd build && cmake .. && make mrcpp_basic mrcpp_ablation energy_estimator"
    fi
}

setup_results_dir() {
    log "Setting up results directory: $RESULTS_DIR"

    # Copy experiments (input data only, skip any generated subdirs)
    if [[ -d "$PROJECT_ROOT/experiments" ]]; then
        mkdir -p "$RESULTS_EXPERIMENTS"
        for src in "$PROJECT_ROOT/experiments"/*/; do
            local env_name
            env_name="$(basename "$src")"
            local dst="$RESULTS_EXPERIMENTS/$env_name"

            if [[ -d "$dst" ]]; then
                log "  experiments/$env_name — already exists, skipping copy"
            else
                log "  Copying experiments/$env_name"
                cp -r "$src" "$dst"
            fi
        done
    else
        log "Warning: $PROJECT_ROOT/experiments not found, skipping experiments copy"
    fi

    # Copy config
    if [[ -d "$PROJECT_ROOT/config" ]]; then
        if [[ -d "$RESULTS_CONFIG" ]]; then
            log "  config/ — already exists, skipping copy"
        else
            log "  Copying config/"
            cp -r "$PROJECT_ROOT/config" "$RESULTS_CONFIG"
        fi
    else
        log "Warning: $PROJECT_ROOT/config not found, skipping config copy"
    fi

    log "Setup complete."
}

run_basic_all() {
    log "=== Running mrcpp_basic on all experiments ==="
    if [[ ! -x "$BASIC_BIN" ]]; then
        log "Error: $BASIC_BIN not found or not executable"; exit 1
    fi

    shopt -s nullglob
    local ran=0
    for exp_dir in "$RESULTS_EXPERIMENTS"/*/; do
        [[ -d "$exp_dir" ]] || continue
        local exp_name
        exp_name="$(basename "$exp_dir")"

        if [[ ! -f "$exp_dir/request.json" || ! -f "$exp_dir/start_point.csv" || ! -f "$exp_dir/mrcpp_basic.xml" ]]; then
            log "  Skip $exp_name (missing request.json, start_point.csv, or mrcpp_basic.xml)"
            continue
        fi

        log "  Running mrcpp_basic: $exp_name"
        "$BASIC_BIN" "$exp_dir"
        ran=1
    done
    [[ "$ran" -eq 1 ]] || log "  No experiment folders found"
}

run_ablation_all() {
    log "=== Running mrcpp_ablation on all experiments ==="
    if [[ ! -x "$ABLAT_BIN" ]]; then
        log "Error: $ABLAT_BIN not found or not executable"; exit 1
    fi

    # Delegate to run_ablation.sh so merge logic is in one place
    bash "$SCRIPT_DIR/run_ablation.sh" "$RESULTS_EXPERIMENTS" "$ABLAT_BIN"
}

run_energy_all() {
    log "=== Running energy_estimator on all mrcpp_results dirs ==="
    if [[ ! -x "$ENERGY_BIN" ]]; then
        log "Error: $ENERGY_BIN not found or not executable"; exit 1
    fi

    shopt -s nullglob
    local ran=0
    for results_dir in "$RESULTS_EXPERIMENTS"/*/mrcpp_results/; do
        [[ -d "$results_dir" ]] || continue
        local exp_name
        exp_name="$(basename "$(dirname "$results_dir")")"

        log "  Energy estimate: $exp_name"
        "$ENERGY_BIN" "$results_dir" --latlon
        ran=1
    done
    [[ "$ran" -eq 1 ]] || log "  No mrcpp_results directories found"
}

# ── main ──────────────────────────────────────────────────────────────────────

main() {
    log "PROJECT_ROOT:  $PROJECT_ROOT"
    log "RESULTS_DIR:   $RESULTS_DIR"
    log "BUILD_DIR:     $BUILD_DIR"

    check_build
    setup_results_dir

    [[ "$RUN_BASIC"    -eq 1 ]] && run_basic_all
    [[ "$RUN_ABLATION" -eq 1 ]] && run_ablation_all
    [[ "$RUN_ENERGY"   -eq 1 ]] && run_energy_all

    log "Done."
}

main "$@"
