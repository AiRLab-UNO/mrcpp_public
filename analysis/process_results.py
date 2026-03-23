#!/usr/bin/env python3
"""
Script to copy CSV files from results folder to result_without_start_end folder
while removing the first and last rows from each CSV file.
"""

import os
import shutil
from pathlib import Path
import csv


def process_csv_file(input_path, output_path):
    """
    Read a CSV file, remove first and last rows, and save to output path.
    
    Args:
        input_path: Path to input CSV file
        output_path: Path to output CSV file
    """
    with open(input_path, 'r') as f:
        lines = f.readlines()
    
    # Remove first and last rows if there are more than 2 rows
    if len(lines) > 2:
        processed_lines = lines[1:-1]
    else:
        # If 2 or fewer rows, result would be empty
        processed_lines = []
    
    with open(output_path, 'w') as f:
        f.writelines(processed_lines)


def copy_and_process_results(source_dir='result_scale_study', dest_dir='scale_result_without_start_end'):
    """
    Copy results folder structure and process all CSV files.
    
    Args:
        source_dir: Source directory name (default: 'results')
        dest_dir: Destination directory name (default: 'result_without_start_end')
    """
    # Get the base path (script directory)
    base_path = Path(__file__).parent
    source_path = base_path / source_dir
    dest_path = base_path / dest_dir
    
    # Check if source directory exists
    if not source_path.exists():
        print(f"Error: Source directory '{source_path}' does not exist!")
        return
    
    # Create destination directory if it doesn't exist
    dest_path.mkdir(exist_ok=True)
    
    # Track statistics
    csv_count = 0
    processed_count = 0
    
    # Walk through the source directory
    for root, dirs, files in os.walk(source_path):
        # Calculate relative path from source
        rel_path = Path(root).relative_to(source_path)
        
        # Create corresponding directory in destination
        dest_subdir = dest_path / rel_path
        dest_subdir.mkdir(parents=True, exist_ok=True)
        
        # Process files in current directory
        for file in files:
            if file.endswith('.csv'):
                csv_count += 1
                input_file = Path(root) / file
                output_file = dest_subdir / file
                
                try:
                    process_csv_file(input_file, output_file)
                    processed_count += 1
                    print(f"Processed: {rel_path / file}")
                except Exception as e:
                    print(f"Error processing {rel_path / file}: {e}")
    
    print(f"\n{'='*60}")
    print(f"Processing complete!")
    print(f"Total CSV files found: {csv_count}")
    print(f"Successfully processed: {processed_count}")
    print(f"Output directory: {dest_path}")
    print(f"{'='*60}")


if __name__ == '__main__':
    copy_and_process_results()
