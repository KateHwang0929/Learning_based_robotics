import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import csv


states = np.load("cartpole_states.npy")
actions = np.load("cartpole_actions.npy")
print("states shape:", states.shape)
print("actions shape:", actions.shape)


#define imitation model
class StudentPolicy(nn.Module):
    def __init__(self):
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(4, 32),   # input: [x, x_dot, theta, theta_dot]
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 2)    # output: 2 actions: left or right
        )

    def forward(self, x):
        return self.net(x)


#train student policy
def train_student(train_states, train_actions, epochs=150, batch_size=64):
    model = StudentPolicy()

    X = torch.tensor(train_states, dtype=torch.float32)
    y = torch.tensor(train_actions, dtype=torch.long)

    dataset = torch.utils.data.TensorDataset(X, y)
    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True
    )

    loss_fn = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    for epoch in range(epochs):
        for batch_X, batch_y in loader:
            logits = model(batch_X)
            loss = loss_fn(logits, batch_y)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

    return model


#evaluate student policy
def choose_action(model, obs):
    obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)

    with torch.no_grad():
        logits = model(obs_tensor)
        action = torch.argmax(logits, dim=1).item()

    return action


def evaluate_student(model, episodes=50):
    env = gym.make("CartPole-v1")

    rewards = []

    for ep in range(episodes):
        obs, info = env.reset(seed=ep)
        done = False
        total_reward = 0

        while not done:
            action = choose_action(model, obs)
            obs, reward, terminated, truncated, info = env.step(action)

            done = terminated or truncated
            total_reward += reward

        rewards.append(total_reward)

    env.close()

    return np.mean(rewards)


#experiment settings
NUM_RUNS = 5
EVAL_EPISODES = 50
EPOCHS = 150
dataset_sizes = list(range(20, 1001, 20))


#Run 5 independent training runs and save one csv
rows = []

for size in dataset_sizes:
    print("\nDataset size:", size)

    run_avg_rewards = []

    for run in range(NUM_RUNS):
        print(f"Run {run + 1}/{NUM_RUNS}")

        rng = np.random.default_rng(42 + run)
        idx = rng.choice(len(states), size=size, replace=False)

        train_states = states[idx]
        train_actions = actions[idx]

        #made it different but reproducible model initialization for each run
        torch.manual_seed(100 + run)

        model = train_student(
            train_states,
            train_actions,
            epochs=EPOCHS
        )

        avg_reward = evaluate_student(
            model,
            episodes=EVAL_EPISODES
        )

        run_avg_rewards.append(avg_reward)

        print(f"Run {run + 1} average reward: {avg_reward:.2f}")

    average_reward = np.mean(run_avg_rewards)

    row = {
        "dataset_size": size,
        "run_1_reward": run_avg_rewards[0],
        "run_2_reward": run_avg_rewards[1],
        "run_3_reward": run_avg_rewards[2],
        "run_4_reward": run_avg_rewards[3],
        "run_5_reward": run_avg_rewards[4],
        "average_reward": average_reward
    }

    rows.append(row)

    print(f"Average of 5 runs: {average_reward:.2f}")


#export csv
output_file = "dataset_size_5run_rewards.csv"
with open(output_file, "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "dataset_size",
            "run_1_reward",
            "run_2_reward",
            "run_3_reward",
            "run_4_reward",
            "run_5_reward",
            "average_reward"
        ]
    )

    writer.writeheader()
    writer.writerows(rows)

print("\nSaved:", output_file)
print("Done.")
