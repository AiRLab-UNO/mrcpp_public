import argparse, csv, sys, math, json, tempfile, time
from pathlib import Path
from typing import List, Tuple, Dict, Optional

try:
    import folium
except Exception:
    print("[ERR] This script requires 'folium'. Install with: pip install folium", file=sys.stderr)
    sys.exit(1)

LatLon = Tuple[float, float]  # (lat, lon)
R_EARTH = 6371000.0

def _to_float(s: str):
    try: return float(s.strip())
    except Exception: return None

def _is_lat(x: float) -> bool: return -90.0 <= x <= 90.0
def _is_lon(x: float) -> bool: return -180.0 <= x <= 180.0

def read_json_data(fp: Path) -> Optional[Dict]:
    try:
        with fp.open("r") as f:
            data = json.load(f)
            if "polygon" not in data or not isinstance(data["polygon"], list):
                return None
            return data
    except Exception:
        return None

def read_two_col_csv(fp: Path, force: str = "auto") -> List[LatLon]:
    rows = []
    with fp.open("r", newline="") as f:
        for r in csv.reader(f):
            if len(r) < 2: continue
            a, b = _to_float(r[0]), _to_float(r[1])
            if a is None or b is None: continue
            rows.append((a, b))
    if not rows: return []

    if force == "latlon":
        return [(a, b) for a, b in rows]
    if force == "lonlat":
        return [(b, a) for a, b in rows]

    a0, b0 = rows[0]
    if _is_lat(a0) and _is_lon(b0):
        return [(a, b) for a, b in rows]
    if _is_lon(a0) and _is_lat(b0):
        return [(b, a) for a, b in rows]
    return [(a, b) for a, b in rows]

def _collect_all_points(tracks: Dict[str, List[LatLon]], json_data: Optional[Dict]) -> List[LatLon]:
    pts = [pt for seq in tracks.values() for pt in seq]
    if json_data and "polygon" in json_data:
        pts.extend([(p[0], p[1]) for p in json_data["polygon"] if isinstance(p, (list, tuple)) and len(p) >= 2])
    if json_data and "obstacles" in json_data:
        for ob in json_data["obstacles"]:
            if isinstance(ob, dict) and "points" in ob:
                pts.extend([(p[0], p[1]) for p in ob["points"] if isinstance(p, (list, tuple)) and len(p) >= 2])
            elif isinstance(ob, list):
                pts.extend([(p[0], p[1]) for p in ob if isinstance(p, (list, tuple)) and len(p) >= 2])
    return pts

def make_map(tracks: Dict[str, List[LatLon]], json_data: Optional[Dict]) -> folium.Map:
    all_pts = _collect_all_points(tracks, json_data)
    if not all_pts:
        print("[ERR] No data to display", file=sys.stderr); sys.exit(6)
    center = (sum(p[0] for p in all_pts)/len(all_pts), sum(p[1] for p in all_pts)/len(all_pts))

    # # Satellite tiles + scale bar
    m = folium.Map(location=center, zoom_start=19, control_scale=True, tiles=None)
    folium.TileLayer(
        # tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        # tiles="OpenStreetMap",
        tiles='ESRIWorldImagery',
        # attr='Imagery &copy; Esri &mdash; Earthstar Geographics, NASA, USGS, NGA, Garmin',
        # name='Esri Antarctic Imagery',
        overlay=False,
        # attr="Tiles courtesy of the U.S. Geological Survey</a>",
        control=False
    ).add_to(m)

    # Polygon (no popups/tooltips)
    if json_data and "polygon" in json_data:
        polygon_coords = [(c[0], c[1]) for c in json_data["polygon"] if isinstance(c, (list, tuple)) and len(c) >= 2]
        if polygon_coords:
            folium.Polygon(
                locations=polygon_coords,
                color="#4ECDC4", weight=1.5,
                fill=True, fillColor="#4ECDC4", fillOpacity=0.2
            ).add_to(m)

    # Obstacles
    if json_data and "obstacles" in json_data:
        for ob in json_data["obstacles"]:
            obstacle = ob.get("points") if isinstance(ob, dict) else ob
            if isinstance(obstacle, list):
                obstacle_coords = [(c[0], c[1]) for c in obstacle if isinstance(c, (list, tuple)) and len(c) >= 2]
                if obstacle_coords:
                    folium.Polygon(
                        locations=obstacle_coords,
                        color="#E63946", weight=1,
                        fill=True, fillColor="#FF0000", fillOpacity=0.35
                    ).add_to(m)

    # Paths + Start/End markers
    colors = ["#A906EF", "#F3A407", "#0DE0D9", "#E6081B", "#1D0AE7", "#0AEF1D",
              "#FFC0CB", "#F405C4", "#07A0F3", "#C5FA05"]

    for idx, (_, pts) in enumerate(tracks.items()):
        if len(pts) < 2: continue
        color = colors[idx % len(colors)]

        folium.PolyLine(
            locations=pts, color=color, weight=1.5, opacity=0.99, smooth_factor=2.0
        ).add_to(m)

        # START (filled circle)
        folium.CircleMarker(
            location=pts[0], radius=7, color="white", weight=0.5,
            fill=True, fill_color=color, fill_opacity=1.0
        ).add_to(m)

        # END (location pin)
        folium.Marker(
            location=pts[-1],
            icon=folium.Icon(color='red', icon='map-marker', prefix='fa')
        ).add_to(m)

    return m

def _save_png_with_selenium(html_path: Path, png_path: Path, size: Tuple[int,int], wait_s: float) -> bool:
    """Render an HTML map to PNG using headless Selenium. Returns True on success."""
    # Try Chrome first, then Firefox
    try:
        from selenium import webdriver
        from selenium.webdriver.chrome.options import Options as ChromeOptions
        opts = ChromeOptions()
        opts.add_argument("--headless=new")
        opts.add_argument(f"--window-size={size[0]},{size[1]}")
        opts.add_argument("--disable-gpu")
        opts.add_argument("--no-sandbox")
        driver = webdriver.Chrome(options=opts)
    except Exception:
        try:
            from selenium import webdriver
            from selenium.webdriver.firefox.options import Options as FFOptions
            opts = FFOptions()
            opts.add_argument("--headless")
            driver = webdriver.Firefox(options=opts)
            driver.set_window_size(size[0], size[1])
        except Exception as e:
            print("[ERR] PNG export requires Selenium + a headless browser (Chrome or Firefox).", file=sys.stderr)
            print("      Install with: pip install selenium webdriver-manager", file=sys.stderr)
            print("      And ensure chromedriver or geckodriver is on PATH.", file=sys.stderr)
            return False

    try:
        driver.get(f"file://{html_path.resolve()}")
        time.sleep(wait_s)  # let tiles/scale load
        driver.save_screenshot(str(png_path))
        return True
    finally:
        driver.quit()

def main():
    ap = argparse.ArgumentParser(
        description="Satellite HTML map from CSV path(s) + JSON polygon/obstacles, with start/end markers, scale bar, and optional PNG export."
    )
    ap.add_argument("path", type=Path, nargs='?', help="CSV file, JSON file, or folder")
    ap.add_argument("--json", type=Path, default=None, help="Optional JSON with polygon/obstacles")
    ap.add_argument("--out", type=Path, default=None, help="Output HTML (default: auto-generated)")
    ap.add_argument("--png", type=Path, default=None, help="Optional PNG output path")
    ap.add_argument("--size", type=str, default="1600x1000", help="PNG size WxH (default: 1600x1000)")
    ap.add_argument("--wait", type=float, default=2.0, help="Seconds to wait for tiles before PNG (default: 2.0)")
    ap.add_argument("--lonlat", action="store_true", help="Force CSV input as (lon,lat)")
    ap.add_argument("--latlon", action="store_true", help="Force CSV input as (lat,lon)")
    args = ap.parse_args()

    if args.lonlat and args.latlon:
        print("[ERR] Choose only one of --lonlat / --latlon", file=sys.stderr); sys.exit(2)
    force = "lonlat" if args.lonlat else ("latlon" if args.latlon else "auto")

    # JSON optional
    json_data = None
    if args.json:
        if not args.json.exists():
            print(f"[ERR] JSON file not found: {args.json}", file=sys.stderr); sys.exit(2)
        json_data = read_json_data(args.json)

    tracks: Dict[str, List[LatLon]] = {}

    if args.path:
        p = args.path
        if not p.exists():
            print(f"[ERR] Not found: {p}", file=sys.stderr); sys.exit(2)

        if p.is_file():
            if p.suffix == '.json':
                json_data = read_json_data(p)
                out_html = args.out if args.out else p.with_name(f"{p.stem}_map.html")
            elif p.suffix == '.csv':
                ll = read_two_col_csv(p, force)
                if len(ll) < 2:
                    print(f"[ERR] Too few points in {p}", file=sys.stderr); sys.exit(3)
                tracks[p.stem] = ll
                out_html = args.out if args.out else p.with_name(f"{p.stem}_paths.html")
            else:
                print(f"[ERR] Unsupported file type: {p.suffix}", file=sys.stderr); sys.exit(2)
        else:
            csvs = sorted(p.glob("*.csv"))
            jsons = list(p.glob("*.json"))
            if jsons and not json_data:
                json_data = read_json_data(jsons[0])
            for fp in csvs:
                ll = read_two_col_csv(fp, force)
                if len(ll) >= 2:
                    tracks[fp.stem] = ll
            out_html = args.out if args.out else p / f"{p.name}_map.html"
    else:
        if json_data:
            out_html = args.out if args.out else args.json.with_name(f"{args.json.stem}_map.html")
        else:
            print("[ERR] No input files provided", file=sys.stderr)
            ap.print_help(); sys.exit(2)

    if not tracks and not json_data:
        print("[ERR] No valid data to display.", file=sys.stderr); sys.exit(5)

    # Build map and save HTML
    m = make_map(tracks, json_data)
    m.save(str(out_html))
    print(f"[OK] Saved HTML: {out_html}")

    # Optional PNG
    if args.png:
        w, h = (int(x) for x in args.size.lower().split("x"))
        # Use the saved HTML for rendering
        ok = _save_png_with_selenium(out_html, args.png, (w, h), args.wait)
        if ok:
            print(f"[OK] Saved PNG: {args.png}")
        else:
            sys.exit(7)

if __name__ == "__main__":
    main()
