import gymnasium as gym
import numpy as np

def expert_policy(obs):
    x, x_dot, theta, theta_dot = obs

    score = theta + 0.5 * theta_dot

    if score > 0:
        return 1   # right
    else:
        return 0   # left


env = gym.make("CartPole-v1")

states = []
actions = []

num_episodes = 20


for episode in range(num_episodes):
    obs, info = env.reset()
    done = False
    total_reward = 0

    while not done:
        action = expert_policy(obs)

        states.append(obs)
        actions.append(action)

        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        total_reward += reward

    print("Episode", episode, "reward:", total_reward)

env.close()

states = np.array(states)
actions = np.array(actions)

print("Dataset states shape:", states.shape)
print("Dataset actions shape:", actions.shape)

np.save("cartpole_states.npy", states)
np.save("cartpole_actions.npy", actions)