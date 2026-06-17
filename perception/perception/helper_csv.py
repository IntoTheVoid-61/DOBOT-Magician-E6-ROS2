import pandas as pd

input_file = "/home/ziga/Desktop/calibration/dataset/poses.csv"
output_file = "/home/ziga/Desktop/calibration/dataset/poses_fil.csv"

# Read whitespace/tab-separated file
df = pd.read_csv(input_file, sep=r"\s+", header=None, engine="python")

# Select columns: 0, 1, 5, 6, 7
filtered = df.iloc[:, [2, 3, 4, 5, 6, 7]]

# Save to new file
filtered.to_csv(output_file, index=False, header=False, sep='\t')

print("Done. Saved to:", output_file)