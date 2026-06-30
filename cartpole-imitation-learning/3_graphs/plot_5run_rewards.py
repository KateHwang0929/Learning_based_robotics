import os
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("dataset_size_5run_rewards.csv")
required_columns = [
    "dataset_size",
    "run_1_reward",
    "run_2_reward",
    "run_3_reward",
    "run_4_reward",
    "run_5_reward",
    "average_reward"
]

for col in required_columns:
    if col not in df.columns:
        raise ValueError(f"Missing required column: {col}")

df = df.sort_values("dataset_size")

print("Loaded:", csv_file)
print(df.head())

# graph
x_col = "dataset_size"

runs = {
    "Run 1": {
        "column": "run_1_reward",
        "color": "red",
        "filename": "graph_1_run1_only.png"
    },
    "Run 2": {
        "column": "run_2_reward",
        "color": "orange",
        "filename": "graph_2_run2_only.png"
    },
    "Run 3": {
        "column": "run_3_reward",
        "color": "green",
        "filename": "graph_3_run3_only.png"
    },
    "Run 4": {
        "column": "run_4_reward",
        "color": "blue",
        "filename": "graph_4_run4_only.png"
    },
    "Run 5": {
        "column": "run_5_reward",
        "color": "purple",
        "filename": "graph_5_run5_only.png"
    },
    "Average": {
        "column": "average_reward",
        "color": "black",
        "filename": "graph_6_average_only.png"
    }
}

output_folder = "reward_graphs"
os.makedirs(output_folder, exist_ok=True)


# Function for each graph
def plot_single_graph(label, column, color, filename):
    plt.figure(figsize=(10, 6))

    plt.plot(
        df[x_col],
        df[column],
        marker="o",
        linewidth=2,
        color=color,
        label=label
    )

    plt.xlabel("Dataset size")
    plt.ylabel("Average reward")
    plt.title(f"{label}: Reward vs Dataset Size")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    save_path = os.path.join(output_folder, filename)
    plt.savefig(save_path, dpi=300)
    plt.close()

    print("Saved:", save_path)


# graph 1-6

for label, info in runs.items():
    plot_single_graph(
        label=label,
        column=info["column"],
        color=info["color"],
        filename=info["filename"]
    )


# graph 7 - all together
plt.figure(figsize=(12, 7))

for label, info in runs.items():
    linewidth = 3 if label == "Average" else 1.8
    alpha = 1.0 if label == "Average" else 0.75

    plt.plot(
        df[x_col],
        df[info["column"]],
        marker="o",
        linewidth=linewidth,
        alpha=alpha,
        color=info["color"],
        label=label
    )

plt.xlabel("Dataset size")
plt.ylabel("Average reward")
plt.title("All Runs and Average: Reward vs Dataset Size")
plt.grid(True)
plt.legend()
plt.tight_layout()
#export and save
save_path = os.path.join(output_folder, "graph_7_all_runs_and_average.png")
plt.savefig(save_path, dpi=300)
plt.close()

print("Saved:", save_path)
print("\nDone. All graphs are inside the folder:", output_folder)