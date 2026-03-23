#!/usr/bin/env python3
"""
Exact TOPPRA-based energy estimator (paper implementation) with lat/lon support.

Core (matches the paper):
  1) Natural cubic spline over arclength s
  2) TOPPRA time-parameterization with component-wise limits:
       |ẋ|,|ẏ| <= v_r   and   |ẍ|,|ÿ| <= a_max
     rest-to-rest boundary when available (compute_trajectory(0,0))
  3) Energy sum:
       E_t = Σ_k [ P(v_k) * Δt_k + max(0, 0.5*m*(v_{k+1}^2 - v_k^2)) ]
     with P(v)=P_h for v < v_r, P(v)=P_r at v≈v_r (no regeneration)

Extras (IO only):
  - Accepts CSV with either X,Y (meters) or lat,lon (degrees) via --latlon
  - If --latlon, converts to local ENU meters using:
      * default origin = first point, or
      * user-specified origin via --ref-lat --ref-lon
"""

import argparse
import os
import sys
import glob
import math
from typing import Tuple, List, Optional

import numpy as np
from scipy.interpolate import CubicSpline

# toppra (required)
try:
    import toppra as ta
    import toppra.constraint as constraint
    import toppra.algorithm as algo
    from toppra.interpolator import SplineInterpolator
except Exception as e:
    print("[ERROR] toppra is required. Install with: pip install toppra", file=sys.stderr)
    raise

# -------------------- I/O --------------------

def read_two_cols_csv(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Read 2-column CSV -> arrays c0, c1 (floats). Supports comma or whitespace, ignores # comments."""
    try:
        data = np.loadtxt(path, delimiter=",", ndmin=2, comments="#", skiprows=1)
        if data.shape[1] < 2:
            raise ValueError("need >=2 columns")
    except Exception:
        data = np.loadtxt(path, ndmin=2, comments="#")
        if data.shape[1] < 2:
            raise ValueError(f"{path}: need at least 2 columns.")
    c0 = data[:, 0].astype(float)
    c1 = data[:, 1].astype(float)
    if len(c0) < 2:
        raise ValueError(f"{path}: not enough points.")
    return c0, c1

def list_csvs(path: str, recursive: bool=False) -> List[str]:
    if os.path.isdir(path):
        pattern = "**/*.csv" if recursive else "*.csv"
        return sorted(glob.glob(os.path.join(path, pattern), recursive=recursive))
    return [path]

# -------------------- lat/lon → local meters --------------------

def deg2rad(d: np.ndarray) -> np.ndarray:
    return d * (math.pi / 180.0)

def latlon_to_local_xy_m(lat_deg: np.ndarray, lon_deg: np.ndarray,
                         ref_lat_deg: Optional[float]=None,
                         ref_lon_deg: Optional[float]=None) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert (lat, lon) [deg] to local ENU meters using an equirectangular approximation around a reference:
      x ≈ East = R * Δλ * cos( mean φ )
      y ≈ North = R * Δφ
    Good for local paths (small areas). R = WGS84 equatorial radius.
    """
    if len(lat_deg) == 0:
        return lat_deg, lon_deg
    R = 6378137.0  # meters
    lat = deg2rad(lat_deg)
    lon = deg2rad(lon_deg)
    if ref_lat_deg is None or ref_lon_deg is None:
        lat0 = lat[0]
        lon0 = lon[0]
    else:
        lat0 = deg2rad(np.array(ref_lat_deg))
        lon0 = deg2rad(np.array(ref_lon_deg))

    dlat = lat - lat0
    dlon = lon - lon0
    lat_mean = 0.5 * (lat + lat0)

    x = R * dlon * np.cos(lat_mean)
    y = R * dlat
    return x, y

# -------------------- geometry & splines --------------------

def arclength_spline(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, CubicSpline, CubicSpline]:
    """Build arclength parameter s and natural cubic splines x(s), y(s)."""
    dx = np.diff(x)
    dy = np.diff(y)
    seg = np.hypot(dx, dy)
    s = np.zeros_like(x)
    s[1:] = np.cumsum(seg)
    # strictly increasing s
    for i in range(1, len(s)):
        if s[i] <= s[i-1]:
            s[i] = s[i-1] + 1e-12
    sx = CubicSpline(s, x, bc_type="natural")
    sy = CubicSpline(s, y, bc_type="natural")
    return s, sx, sy

def build_path_interpolator_from_spline(s: np.ndarray, sx: CubicSpline, sy: CubicSpline, samples: int=200) -> SplineInterpolator:
    """Create a toppra SplineInterpolator r(u) over u∈[0,1], sampled from x(s), y(s)."""
    uu = np.linspace(0, 1, samples)
    L = s[-1] if s[-1] > 0 else 1.0
    ss = uu * L
    X = sx(ss)
    Y = sy(ss)
    way_pts = np.vstack([X, Y]).T  # (N,2)
    return SplineInterpolator(uu, way_pts)

# -------------------- pen-and-paper (v_r, P_h, P_r) --------------------

def pen_and_paper(drone_mass: float,
                  drone_area_m2: float,
                  prop_radius_m: float,
                  n_props: int,
                  c0: float, c1: float, c2: float,
                  air_density: float=1.225,
                  g: float=9.8,
                  prop_eff: float=0.391,
                  range_coeff: float=1.092) -> Tuple[float, float, float]:
    """
    Compute (v_r, P_h, P_r) exactly as in your C++ EnergyCalculator.
    """
    # induced velocity at hover
    v_i_h = math.sqrt((drone_mass * g) / (2 * air_density * math.pi * prop_radius_m**2 * n_props))
    # hover power
    P_h = math.sqrt((drone_mass * g) ** 3) / (
        prop_eff * prop_radius_m * math.sqrt(2 * air_density * math.pi * n_props)
    )
    P_r = P_h * range_coeff
    # best-speed model (area in cm^2)
    v_r_inv = c0 + c1 * v_i_h + c2 * (drone_area_m2 * 10000.0)
    v_r = v_i_h / v_r_inv
    return v_r, P_h, P_r

# -------------------- TOPPRA time-parameterization --------------------

def toppra_time_parameterize(path: SplineInterpolator,
                             v_r: float,
                             a_max: float,
                             gridpoints: Optional[np.ndarray]=None,
                             time_samples: int=1000) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Component-wise bounds: |ẋ|,|ẏ| <= v_r and |ẍ|,|ÿ| <= a_max.
    Prefer rest-to-rest (compute_trajectory(0,0)), else fallback.
    Returns (t, q(t), q̇(t)).
    """
    dof = path.dof  # expect 2

    vlims = np.vstack([-v_r * np.ones(dof),  v_r * np.ones(dof)]).T  # (dof,2)
    alims = np.vstack([-a_max * np.ones(dof), a_max * np.ones(dof)]).T

    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(
        alims,
        discretization_scheme=constraint.DiscretizationType.Interpolation
    )

    if gridpoints is None:
        gridpoints = np.linspace(0, 1, 200)

    instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints)

    # Rest-to-rest if your toppra build supports it
    try:
        jnt_traj = instance.compute_trajectory(0, 0)
    except TypeError:
        jnt_traj = instance.compute_trajectory()

    if jnt_traj is None:
        raise RuntimeError("TOPPRA failed (try different --amax, check path).")

    T = jnt_traj.duration
    ts = np.linspace(0, T, time_samples)
    qs = jnt_traj(ts)       # (N,2)
    qds = jnt_traj(ts, 1)   # (N,2)
    return ts, qs, qds

# -------------------- Energy (paper-exact) --------------------

def energy_exact(ts: np.ndarray,
                 qds: np.ndarray,
                 mass_kg: float,
                 v_r: float,
                 P_h: float,
                 P_r: float) -> Tuple[float, float]:
    """
    E_t = Σ_k [ P(v_k)*Δt_k + max(0, 0.5*m*(v_{k+1}^2 - v_k^2)) ]
    with P= P_h for v < v_r, else P_r (at v≈v_r). No regeneration.
    Returns (Joules, Watt-hours).
    """
    speeds = np.linalg.norm(qds, axis=1)
    E = 0.0
    for k in range(len(ts) - 1):
        dt = ts[k+1] - ts[k]
        if dt <= 0:
            continue
        v0 = speeds[k]
        v1 = speeds[k+1]
        vavg = 0.5 * (v0 + v1)

        P_elec = P_h if vavg < v_r - 1e-9 else P_r
        E += P_elec * dt

        dKE = 0.5 * mass_kg * (v1*v1 - v0*v0)
        if dKE > 0:
            E += dKE
    return E, E / 3600.0

# -------------------- Batch wrapper --------------------

def process_one(csv_path: str,
                use_latlon: bool,
                ref_lat: Optional[float],
                ref_lon: Optional[float],
                mass: float, area: float, prop_radius: float, n_props: int,
                c0: float, c1: float, c2: float,
                a_max: float,
                samples: int,
                time_samples: int) -> Tuple[str, float, float, float, float]:
    """
    Returns (filename, length_m, time_s, E[J], E[Wh]).
    """
    c0col, c1col = read_two_cols_csv(csv_path)

    if use_latlon:
        x, y = latlon_to_local_xy_m(c0col, c1col, ref_lat, ref_lon)
    else:
        x, y = c0col, c1col

    s, sx, sy = arclength_spline(x, y)
    L = float(s[-1])

    # pen-and-paper parameters
    v_r, P_h, P_r = pen_and_paper(mass, area, prop_radius, n_props, c0, c1, c2)

    # build path & time-param
    path = build_path_interpolator_from_spline(s, sx, sy, samples=samples)
    ts, qs, qds = toppra_time_parameterize(path, v_r=v_r, a_max=a_max, time_samples=time_samples)

    # energy
    E_J, E_Wh = energy_exact(ts, qds, mass, v_r, P_h, P_r)

    return (os.path.basename(csv_path), L, float(ts[-1]), E_J, E_Wh)

# -------------------- CLI --------------------

def main():
    ap = argparse.ArgumentParser(description="Exact TOPPRA energy from CSV; supports lat/lon input.")
    ap.add_argument("path", help="CSV file or directory of CSV files.")
    ap.add_argument("--recursive", action="store_true", help="Recurse into subdirectories if path is a directory.")

    # Input mode
    ap.add_argument("--latlon", action="store_true",
                    help="Interpret CSV columns as (lat, lon) in degrees; convert to local meters.")
    ap.add_argument("--ref-lat", type=float, default=None, help="Reference latitude in deg for ENU origin (optional).")
    ap.add_argument("--ref-lon", type=float, default=None, help="Reference longitude in deg for ENU origin (optional).")

    # UAV & model (defaults match your C++)
    ap.add_argument("--mass", type=float, default=3.2, help="UAV mass [kg].")
    ap.add_argument("--area", type=float, default=0.07, help="Projected area [m^2].")
    ap.add_argument("--prop-radius", type=float, default=0.19, help="Propeller radius [m].")
    ap.add_argument("--n-props", type=int, default=4, help="Number of propellers.")
    ap.add_argument("--c0", type=float, default=0.041546, help="Best-speed model c0.")
    ap.add_argument("--c1", type=float, default=0.041122, help="Best-speed model c1.")
    ap.add_argument("--c2", type=float, default=0.00053292, help="Best-speed model c2.")

    # motion bounds & sampling
    ap.add_argument("--amax", type=float, default=2.0, help="Acceleration bound [m/s^2] (component-wise).")
    ap.add_argument("--samples", type=int, default=200, help="Path points to build SplineInterpolator.")
    ap.add_argument("--time-samples", type=int, default=1000, help="Time samples for integration.")

    args = ap.parse_args()

    if args.latlon and ((args.ref_lat is None) ^ (args.ref_lon is None)):
        print("[ERROR] Provide both --ref-lat and --ref-lon or neither.", file=sys.stderr)
        sys.exit(2)

    csvs = list_csvs(args.path, args.recursive)
    if not csvs:
        print(f"No CSV files found under: {args.path}", file=sys.stderr)
        sys.exit(2)

    header = f"(mass={args.mass} kg, a_max={args.amax} m/s^2)  input={'lat/lon' if args.latlon else 'meters'}"
    if args.latlon and args.ref_lat is not None:
        header += f"  ENU origin=({args.ref_lat:.6f},{args.ref_lon:.6f})"
    print(header)
    print("File                                Time[s]        E[J]        E[Wh]")
    print("------------------------------------------------------------------")

    sumE = sumWh = sumL = sumT = 0.0
    worstE = -1.0
    worstFn = ""

    for p in csvs:
        try:
            fn, L, T, EJ, EWh = process_one(
                p, args.latlon, args.ref_lat, args.ref_lon,
                args.mass, args.area, args.prop_radius, args.n_props,
                args.c0, args.c1, args.c2,
                args.amax, args.samples, args.time_samples
            )
            sumE += EJ; sumWh += EWh; sumL += L; sumT += T
            if EJ > worstE: worstE, worstFn = EJ, fn
            print(f"{fn:32s}   {T:8.2f}   {EJ:12.2f}   {EWh:10.4f}")
        except Exception as e:
            print(f"[SKIP] {p}: {e}", file=sys.stderr)

    print("------------------------------------------------------------------")
    print(f"Fleet sum: E={sumE:.2f} J  ({sumWh:.4f} Wh)   Time={sumT:.2f} s")
    if worstE >= 0:
        print(f"Worst file: {worstFn}  E={worstE:.2f} J ({worstE/3600.0:.4f} Wh)")


if __name__ == "__main__":
    main()
