#!/usr/bin/env python3
"""
Script to combine multiple CSV route files into a specified number of output files.
Merges path points from multiple routes to create more efficient consolidated routes.
"""

import os
import sys
from pathlib import Path
import pandas as pd
import argparse
import numpy as np

# Optional import for spatial clustering
try:
    from sklearn.cluster import KMeans
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


def load_csv_files(directory):
    """
    Load all CSV files from the specified directory.
    
    Args:
        directory: Path to directory containing CSV files
        
    Returns:
        Dictionary with filename as key and dataframe as value
    """
    csv_data = {}
    directory_path = Path(directory)
    
    if not directory_path.exists():
        print(f"Error: Directory '{directory}' does not exist!")
        return csv_data
    
    # Find all CSV files
    csv_files = list(directory_path.glob('*.csv'))
    
    if not csv_files:
        print(f"No CSV files found in '{directory}'")
        return csv_data
    
    # Load each CSV file
    for csv_file in sorted(csv_files):
        try:
            df = pd.read_csv(csv_file, header=None, names=['lat', 'lon'])
            csv_data[csv_file.stem] = df
            print(f"Loaded: {csv_file.name} ({len(df)} points)")
        except Exception as e:
            print(f"Error loading {csv_file.name}: {e}")
    
    return csv_data


def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points (lat, lon)."""
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def find_closest_route(current_end, remaining_routes):
    """Find the route with the closest starting point to current endpoint."""
    min_dist = float('inf')
    closest_idx = 0
    flip = False
    
    for idx, df in enumerate(remaining_routes):
        start_point = (df['lat'].iloc[0], df['lon'].iloc[0])
        end_point = (df['lat'].iloc[-1], df['lon'].iloc[-1])
        
        # Check distance to start
        dist_to_start = calculate_distance(current_end, start_point)
        # Check distance to end (we can flip the route)
        dist_to_end = calculate_distance(current_end, end_point)
        
        if dist_to_start < min_dist:
            min_dist = dist_to_start
            closest_idx = idx
            flip = False
        
        if dist_to_end < min_dist:
            min_dist = dist_to_end
            closest_idx = idx
            flip = True
    
    return closest_idx, flip, min_dist


def combine_routes_round_robin(csv_data, num_outputs):
    """
    Combine routes using optimized chaining.
    Creates efficient paths by connecting nearby routes and minimizing backtracking.
    
    Args:
        csv_data: Dictionary of dataframes
        num_outputs: Number of output files to create
        
    Returns:
        List of combined dataframes
    """
    if num_outputs >= len(csv_data):
        print(f"Warning: Number of outputs ({num_outputs}) >= number of inputs ({len(csv_data)})")
        print("Returning original files...")
        return list(csv_data.values())
    
    files_list = [(name, df.copy()) for name, df in csv_data.items()]
    combined_routes = []
    
    # Distribute routes into groups
    routes_per_output = len(files_list) // num_outputs
    remainder = len(files_list) % num_outputs
    
    start_idx = 0
    for output_idx in range(num_outputs):
        # Determine how many routes go into this output
        num_routes = routes_per_output + (1 if output_idx < remainder else 0)
        end_idx = start_idx + num_routes
        
        if start_idx >= len(files_list):
            break
        
        # Get routes for this output
        group_routes = [df.copy() for name, df in files_list[start_idx:end_idx]]
        group_names = [name for name, df in files_list[start_idx:end_idx]]
        
        if not group_routes:
            continue
        
        print(f"\nCombining into route {output_idx + 1}:")
        
        # Start with the first route
        combined_df = group_routes[0]
        current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
        print(f"  Starting with: {group_names[0]} ({len(group_routes[0])} points)")
        
        remaining_routes = group_routes[1:]
        remaining_names = group_names[1:]
        
        # Chain remaining routes by proximity
        while remaining_routes:
            closest_idx, flip, dist = find_closest_route(current_end, remaining_routes)
            
            next_route = remaining_routes[closest_idx]
            next_name = remaining_names[closest_idx]
            
            # Flip route if needed for better connection
            if flip:
                next_route = next_route.iloc[::-1].reset_index(drop=True)
                print(f"  + {next_name} (flipped, dist: {dist:.6f}, {len(next_route)} points)")
            else:
                print(f"  + {next_name} (dist: {dist:.6f}, {len(next_route)} points)")
            
            # Combine routes
            combined_df = pd.concat([combined_df, next_route], ignore_index=True)
            current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
            
            # Remove used route
            remaining_routes.pop(closest_idx)
            remaining_names.pop(closest_idx)
        
        combined_routes.append(combined_df)
        print(f"  Total: {len(combined_df)} points")
        
        start_idx = end_idx
    
    return combined_routes


def combine_routes_sequential(csv_data, num_outputs):
    """
    Combine routes by spatial proximity clustering.
    Groups all points using K-means and assigns routes to clusters.
    
    Args:
        csv_data: Dictionary of dataframes
        num_outputs: Number of output files to create
        
    Returns:
        List of combined dataframes
    """
    if num_outputs >= len(csv_data):
        print(f"Warning: Number of outputs ({num_outputs}) >= number of inputs ({len(csv_data)})")
        print("Returning original files...")
        return list(csv_data.values())
    
    print("\nClustering all points into regions...")
    
    # Collect all points with route information
    all_points = []
    point_sources = []
    
    for filename, df in csv_data.items():
        for _, row in df.iterrows():
            all_points.append([row['lat'], row['lon']])
            point_sources.append(filename)
    
    all_points = np.array(all_points)
    
    # Simple clustering based on geographic regions
    # Divide points into num_outputs clusters
    from scipy.cluster.vq import kmeans2
    
    try:
        centroids, labels = kmeans2(all_points, num_outputs, minit='points')
    except:
        # Fallback to simple spatial division
        lat_bins = np.linspace(all_points[:, 0].min(), all_points[:, 0].max(), num_outputs + 1)
        labels = np.digitize(all_points[:, 0], lat_bins) - 1
        labels = np.clip(labels, 0, num_outputs - 1)
    
    # Group routes by which cluster they belong to most
    route_clusters = {}
    for filename, df in csv_data.items():
        route_points_mask = [ps == filename for ps in point_sources]
        route_labels = labels[route_points_mask]
        
        # Assign route to cluster with most points
        cluster_id = np.bincount(route_labels).argmax()
        
        if cluster_id not in route_clusters:
            route_clusters[cluster_id] = []
        route_clusters[cluster_id].append((filename, df.copy()))
        
        print(f"  {filename} -> cluster {cluster_id + 1}")
    
    # Combine routes within each cluster efficiently
    combined_routes = []
    for cluster_id in sorted(route_clusters.keys()):
        routes_in_cluster = route_clusters[cluster_id]
        
        print(f"\nOptimizing cluster {cluster_id + 1} ({len(routes_in_cluster)} routes):")
        
        # Start with first route
        combined_df = routes_in_cluster[0][1]
        current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
        print(f"  Starting with: {routes_in_cluster[0][0]} ({len(combined_df)} points)")
        
        remaining = routes_in_cluster[1:]
        
        # Chain by proximity
        while remaining:
            remaining_dfs = [df for name, df in remaining]
            remaining_names = [name for name, df in remaining]
            
            closest_idx, flip, dist = find_closest_route(current_end, remaining_dfs)
            
            next_route = remaining_dfs[closest_idx]
            next_name = remaining_names[closest_idx]
            
            if flip:
                next_route = next_route.iloc[::-1].reset_index(drop=True)
                print(f"  + {next_name} (flipped, dist: {dist:.6f}, {len(next_route)} points)")
            else:
                print(f"  + {next_name} (dist: {dist:.6f}, {len(next_route)} points)")
            
            combined_df = pd.concat([combined_df, next_route], ignore_index=True)
            current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
            
            remaining.pop(closest_idx)
        
        combined_routes.append(combined_df)
        print(f"  Total: {len(combined_df)} points")
    
    return combined_routes


def combine_routes_spatial(csv_data, num_outputs):
    """
    Combine routes using greedy nearest-neighbor approach.
    Builds each output route by always adding the closest unassigned route.
    
    Args:
        csv_data: Dictionary of dataframes
        num_outputs: Number of output files to create
        
    Returns:
        List of combined dataframes
    """
    if num_outputs >= len(csv_data):
        print(f"Warning: Number of outputs ({num_outputs}) >= number of inputs ({len(csv_data)})")
        print("Returning original files...")
        return list(csv_data.values())
    
    files_list = [(name, df.copy()) for name, df in csv_data.items()]
    combined_routes = []
    used_indices = set()
    
    # Calculate target routes per output
    routes_per_output = len(files_list) // num_outputs
    remainder = len(files_list) % num_outputs
    
    for output_idx in range(num_outputs):
        target_routes = routes_per_output + (1 if output_idx < remainder else 0)
        
        print(f"\nBuilding combined route {output_idx + 1} (target: {target_routes} routes):")
        
        # Find an unused starting route (prefer centrally located ones)
        available_indices = [i for i in range(len(files_list)) if i not in used_indices]
        if not available_indices:
            break
        
        # Start with first available route
        start_idx = available_indices[0]
        combined_df = files_list[start_idx][1]
        current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
        used_indices.add(start_idx)
        print(f"  Starting with: {files_list[start_idx][0]} ({len(combined_df)} points)")
        
        # Greedily add closest routes
        for _ in range(target_routes - 1):
            available_indices = [i for i in range(len(files_list)) if i not in used_indices]
            if not available_indices:
                break
            
            # Find closest route
            min_dist = float('inf')
            best_idx = None
            best_flip = False
            
            for idx in available_indices:
                df = files_list[idx][1]
                start_point = (df['lat'].iloc[0], df['lon'].iloc[0])
                end_point = (df['lat'].iloc[-1], df['lon'].iloc[-1])
                
                dist_to_start = calculate_distance(current_end, start_point)
                dist_to_end = calculate_distance(current_end, end_point)
                
                if dist_to_start < min_dist:
                    min_dist = dist_to_start
                    best_idx = idx
                    best_flip = False
                
                if dist_to_end < min_dist:
                    min_dist = dist_to_end
                    best_idx = idx
                    best_flip = True
            
            if best_idx is not None:
                next_route = files_list[best_idx][1].copy()
                next_name = files_list[best_idx][0]
                
                if best_flip:
                    next_route = next_route.iloc[::-1].reset_index(drop=True)
                    print(f"  + {next_name} (flipped, dist: {min_dist:.6f}, {len(next_route)} points)")
                else:
                    print(f"  + {next_name} (dist: {min_dist:.6f}, {len(next_route)} points)")
                
                combined_df = pd.concat([combined_df, next_route], ignore_index=True)
                current_end = (combined_df['lat'].iloc[-1], combined_df['lon'].iloc[-1])
                used_indices.add(best_idx)
        
        combined_routes.append(combined_df)
        print(f"  Total: {len(combined_df)} points")
    
    return combined_routes


def save_combined_routes(combined_routes, output_dir, prefix="combined_route"):
    """
    Save combined routes to output directory.
    
    Args:
        combined_routes: List of dataframes
        output_dir: Output directory path
        prefix: Prefix for output filenames
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    for i, df in enumerate(combined_routes):
        output_file = output_path / f"{prefix}_{i+1}.csv"
        df.to_csv(output_file, index=False, header=False)
        print(f"Saved: {output_file} ({len(df)} points)")


def main():
    parser = argparse.ArgumentParser(
        description='Combine multiple CSV route files into fewer consolidated routes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Combination Methods:
  optimized    : Chain routes by proximity, minimizing distances (default)
  clustered    : Cluster points into regions, then optimize each region
  greedy       : Greedily build routes by always adding nearest neighbor

Examples:
  # Combine 8 routes into 3 using optimized chaining
  python combine_routes.py results/cape/popcorn -n 3 -o output/cape

  # Combine routes with greedy nearest-neighbor
  python combine_routes.py results/simple/mrcpp -n 2 -m greedy -o output/simple

  # Cluster-based combination
  python combine_routes.py result_without_start_end/complex/darp -n 2 -m clustered
        """
    )
    
    parser.add_argument('directory', type=str,
                       help='Directory containing CSV files to combine')
    parser.add_argument('-n', '--num-outputs', type=int, required=True,
                       help='Number of output combined route files')
    parser.add_argument('-o', '--output', type=str, default='combined_routes',
                       help='Output directory (default: combined_routes)')
    parser.add_argument('-m', '--method', type=str, 
                       choices=['optimized', 'clustered', 'greedy'],
                       default='optimized',
                       help='Method for combining routes (default: optimized)')
    parser.add_argument('-p', '--prefix', type=str, default='combined_route',
                       help='Prefix for output filenames (default: combined_route)')
    
    args = parser.parse_args()
    
    print(f"Loading CSV files from: {args.directory}")
    print("=" * 60)
    
    # Load CSV files
    csv_data = load_csv_files(args.directory)
    
    if not csv_data:
        print("No data to process. Exiting.")
        return
    
    print("=" * 60)
    print(f"Total files loaded: {len(csv_data)}")
    print(f"Target number of outputs: {args.num_outputs}")
    print(f"Combination method: {args.method}")
    print("=" * 60)
    
    # Combine routes based on selected method
    if args.method == 'optimized':
        combined_routes = combine_routes_round_robin(csv_data, args.num_outputs)
    elif args.method == 'clustered':
        combined_routes = combine_routes_sequential(csv_data, args.num_outputs)
    elif args.method == 'greedy':
        combined_routes = combine_routes_spatial(csv_data, args.num_outputs)
    
    # Save combined routes
    print("=" * 60)
    print("Saving combined routes...")
    save_combined_routes(combined_routes, args.output, args.prefix)
    
    print("=" * 60)
    print(f"Done! Combined {len(csv_data)} files into {len(combined_routes)} output files")
    print(f"Output directory: {args.output}")


if __name__ == '__main__':
    main()
