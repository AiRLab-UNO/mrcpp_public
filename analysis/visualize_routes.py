#!/usr/bin/env python3
"""
Script to visualize CSV route files from a given directory.
Each CSV file is plotted as a path with the filename as the legend.
"""

import os
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd
import argparse


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


def visualize_routes(csv_data, title=None, save_path=None):
    """
    Visualize all routes on a single plot.
    
    Args:
        csv_data: Dictionary with filename as key and dataframe as value
        title: Optional title for the plot
        save_path: Optional path to save the figure
    """
    if not csv_data:
        print("No data to visualize!")
        return
    
    plt.figure(figsize=(12, 8))
    
    # Plot each route
    for filename, df in csv_data.items():
        plt.plot(df['lon'].values, df['lat'].values, marker='o', markersize=3, 
                label=filename, linewidth=2, alpha=0.7)
        
        # Mark start and end points
        plt.plot(df['lon'].iloc[0], df['lat'].iloc[0], 'o', 
                markersize=8, markeredgecolor='black', markeredgewidth=1.5)
        plt.plot(df['lon'].iloc[-1], df['lat'].iloc[-1], 's', 
                markersize=8, markeredgecolor='black', markeredgewidth=1.5)
    
    plt.xlabel('Longitude', fontsize=12)
    plt.ylabel('Latitude', fontsize=12)
    plt.title(title if title else 'Route Visualization', fontsize=14, fontweight='bold')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Save figure if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize CSV route files from a directory',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python visualize_routes.py results/simple/popcorn
  python visualize_routes.py result_without_start_end/cape/mrcpp --title "Cape MRCPP Routes"
  python visualize_routes.py results/complex/darp --save output.png
        """
    )
    
    parser.add_argument('directory', type=str, 
                       help='Directory containing CSV files to visualize')
    parser.add_argument('--title', '-t', type=str, default=None,
                       help='Title for the plot')
    parser.add_argument('--save', '-s', type=str, default=None,
                       help='Path to save the figure (e.g., output.png)')
    
    args = parser.parse_args()
    
    print(f"Loading CSV files from: {args.directory}")
    print("=" * 60)
    
    # Load CSV files
    csv_data = load_csv_files(args.directory)
    
    if csv_data:
        print("=" * 60)
        print(f"Total files loaded: {len(csv_data)}")
        print("=" * 60)
        
        # Visualize
        visualize_routes(csv_data, title=args.title, save_path=args.save)
    else:
        print("No data to visualize. Exiting.")


if __name__ == '__main__':
    main()
