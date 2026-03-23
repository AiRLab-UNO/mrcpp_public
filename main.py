import os
import re
import subprocess
import csv
from pathlib import Path
from time import time
import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'scripts'))


class Benchmark:
    def __init__(self, env_name: str):
        self.env_name = env_name
        self.env_path = f"environments/{env_name}"
        self.json_pdef_path = f"{self.env_path}/request.json"
        self.yaml_config_path = f"{self.env_path}/config.yaml"
        self.start_point_path = f"{self.env_path}/start_point.csv"

    def estimate_energy(self, mrcpp_outdir, append_initial_location=False):

        if append_initial_location:
            with open(self.start_point_path) as f:
                reader = csv.reader(f)
                start_row = next(reader)
                lat, lon = float(start_row[0]), float(start_row[1])
                print(f"Initial location: lat={lat}, lon={lon}")

            start_point = [str(lat), str(lon)]

            for csv_file in sorted(Path(mrcpp_outdir).glob("*.csv")):
                with open(csv_file) as f:
                    reader = csv.reader(f)
                    header = next(reader)
                    rows = list(reader)
                with open(csv_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                    writer.writerow(start_point)
                    writer.writerows(rows)
                    writer.writerow(start_point)
                print(f"Updated {csv_file.name}: prepended and appended initial location")

        energy_estimator = "executables/energy_estimator"
        cmd = [energy_estimator, mrcpp_outdir, "--latlon"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        output = result.stdout + result.stderr
        return output 



def run_benchmarks():
    benchmarks = ["cape", "complex", "island", "rect", "simple"]
    for env_name in benchmarks:
        benchmark = Benchmark(env_name)
        mrcpp_outdir = f"results/{env_name}/mrcpp"
        benchmark(mrcpp_outdir, True)


def main():
    # benchmarks = ["cape", "complex_12","complex_22", "island", "rect", "simple", "wetland","nice"]
    
    benchmarks = ["complex_4", "complex_5", "complex_6", "complex_7", "complex_8", "complex_9", "complex_10","cape_4","cape_6","cape_8","cape_10"]
    # methods = ["eamcmp", "mrcpp", "popcorn", "darp"]
    methods = ["eamcmp", "mrcpp"]
    # methods = ["popcorn"]
    
    for env_name in benchmarks:
        benchmark = Benchmark(env_name)
        for method in methods:
            results_dir = f"planner_data/scale_result_with_start_end/{env_name}/{method}"
            
            # Check if the method directory exists and has CSV files
            if not os.path.exists(results_dir):
                print(f"Skipping {env_name}/{method} - directory not found")
                continue
            
            csv_files = list(Path(results_dir).glob("*.csv"))
            if not csv_files:
                print(f"Skipping {env_name}/{method} - no CSV files found")
                continue
            
            print(f"\nProcessing {env_name}/{method}...")
            energy_estimation = benchmark.estimate_energy(results_dir)
            
            with open(f"{results_dir}/energy_estimation_with_initial_location.txt", 'w') as f:
                f.write(energy_estimation)
                


if __name__ == "__main__":
    
    main()