#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROOT_DIR="${1:-$PROJECT_ROOT/results/experiments}"
ABLAT_BIN="${2:-$PROJECT_ROOT/build/mrcpp_ablation}"

log() {
    echo "[run_ablation] $*"
}

check_prereqs() {
    if [[ ! -x "$ABLAT_BIN" ]]; then
        log "Error: binary not executable: $ABLAT_BIN"
        exit 1
    fi
    if [[ ! -d "$ROOT_DIR" ]]; then
        log "Error: experiments directory not found: $ROOT_DIR"
        exit 1
    fi
}

run_one_experiment() {
    local exp_dir="$1"
    local exp_name
    local legacy_dir
    local ablation_dir
    exp_name="$(basename "$exp_dir")"
    legacy_dir="$exp_dir/mrcpp_results"
    ablation_dir="$exp_dir/mrcpp_ablation"

    if [[ ! -f "$exp_dir/request.json" || ! -f "$exp_dir/start_point.csv" ]]; then
        log "Skip $exp_name (missing request.json or start_point.csv)"
        return
    fi

    log "Processing $exp_name"
    "$ABLAT_BIN" "$exp_dir"

    if [[ -d "$legacy_dir" ]]; then
        mkdir -p "$ablation_dir"
        cp -f "$legacy_dir"/ablation_*.csv "$ablation_dir"/ 2>/dev/null || true
        rm -rf "$legacy_dir"
        log "Migrated legacy output to $ablation_dir"
    fi
}

run_all_experiments() {
    shopt -s nullglob
    local dirs=("$ROOT_DIR"/*)
    local dir
    local ran=0

    for dir in "${dirs[@]}"; do
        [[ -d "$dir" ]] || continue
        run_one_experiment "$dir"
        ran=1
    done

    if [[ "$ran" -eq 0 ]]; then
        log "No experiment folders found in $ROOT_DIR"
    fi
}

merge_study_csv() {
    local study="$1"
    local output_file="$ROOT_DIR/ablation_${study}_all.csv"
    local first=1
    local f

    : > "$output_file"
    shopt -s nullglob
    for f in "$ROOT_DIR"/*/mrcpp_ablation/ablation_${study}.csv; do
        if [[ "$first" -eq 1 ]]; then
            cat "$f" >> "$output_file"
            first=0
        else
            tail -n +2 "$f" >> "$output_file"
        fi
    done

    if [[ "$first" -eq 1 ]]; then
        rm -f "$output_file"
        log "No ablation_${study}.csv files found to merge"
    else
        log "Merged: $output_file"
    fi
}

merge_all_csvs() {
    merge_study_csv "orientation"
    merge_study_csv "headland"
    merge_study_csv "transition"
}

main() {
    log "ROOT_DIR: $ROOT_DIR"
    log "ABLAT_BIN: $ABLAT_BIN"
    check_prereqs
    run_all_experiments
    merge_all_csvs
}

main "$@"

rm amtsp*