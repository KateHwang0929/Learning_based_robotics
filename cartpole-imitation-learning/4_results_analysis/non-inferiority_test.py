import pandas as pd
import numpy as np
from scipy import stats

# Load CSV
df = pd.read_csv("dataset_size_5run_rewards.csv")

run_cols = [
    "run_1_reward",
    "run_2_reward",
    "run_3_reward",
    "run_4_reward",
    "run_5_reward"
]


df["mean_reward"] = df[run_cols].mean(axis=1)
df["std_reward"] = df[run_cols].std(axis=1, ddof=1)
df["sem_reward"] = df["std_reward"] / np.sqrt(len(run_cols))

# 95% confidence interval for each dataset size
n = len(run_cols)
t_crit = stats.t.ppf(0.975, df=n - 1)

df["ci_lower"] = df["mean_reward"] - t_crit * df["sem_reward"]
df["ci_upper"] = df["mean_reward"] + t_crit * df["sem_reward"]

best_mean = df["mean_reward"].max()

# Non-inferiority margin
# 25 reward points = 5% of CartPole max reward 500
delta = 25


df["non_inferior"] = df["ci_lower"] >= best_mean - delta
candidates = df[df["non_inferior"]]

if len(candidates) > 0:
    optimal_row = candidates.sort_values("dataset_size").iloc[0]
else:
    optimal_row = df.sort_values("mean_reward", ascending=False).iloc[0]

print("Best observed mean reward:", best_mean)
print("Non-inferiority margin:", delta)
print()
print("Optimal dataset size:", int(optimal_row["dataset_size"]))
print("Mean reward:", optimal_row["mean_reward"])
print("95% CI:", optimal_row["ci_lower"], "to", optimal_row["ci_upper"])
print("Standard deviation:", optimal_row["std_reward"])

# save it as csv
df.to_csv("dataset_size_statistical_summary.csv", index=False)
print("\nSaved: dataset_size_statistical_summary.csv")