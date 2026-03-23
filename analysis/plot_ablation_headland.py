import glob
import os
import pandas as pd
import matplotlib.pyplot as plt

# Recursively find all ablation_headland.csv files
pattern = os.path.join(os.path.dirname(__file__), "..", "results", "experiments", "*", "mrcpp_ablation", "ablation_headland.csv")

csv_files = sorted(glob.glob(pattern))

if not csv_files:
    raise FileNotFoundError(f"No ablation_headland.csv files found matching: {pattern}")

fig, ax = plt.subplots(figsize=(10, 6))

for csv_path in csv_files:
    print(csv_path)
    # Extract environment name from path: experiments/<env>/mrcpp_ablation/ablation_headland.csv
    env_name = csv_path.split(os.sep)[-3]

    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.strip()

    # Sort by headland_scale for clean line plots
    df = df.sort_values("headland_scale")

    ax.plot(df["headland_scale"], df["Et_Wh"], marker="o", label=env_name)

ax.set_xlabel("Buffer Scale")
ax.set_ylabel("$E_t$ (Wh)")
# ax.set_title("Headland Scale vs. Energy Consumption ($E_t$)")
ax.legend(title="Environment")
ax.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
output_path = os.path.join(os.path.dirname(__file__), "ablation_headland_plot.png")
plt.savefig(output_path, dpi=150)
print(f"Plot saved to: {output_path}")
plt.show()
