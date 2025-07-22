import pygame
import gymnasium as gym
import numpy as np
from q_table_agent import BlackjackAgent
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
from matplotlib.pyplot import ylabel
import os.path
import time

env = gym.make(id='Blackjack-v1')
ACTION_SPACE = env.action_space.n
LEARNING_RATE = 0.001
DISCOUNT_FACTOR = 0.95
EPSILON = 0.1

agent = BlackjackAgent(env= env, learning_rate= LEARNING_RATE, discount_factor= DISCOUNT_FACTOR, epsilon= EPSILON)  #TODO

current_path = os.path.dirname(os.path.realpath(__file__))
timestamp = time.strftime("%Y%m%d%H%M%S")

NUM_EPISODE = 500000
PRINT_INTERVAL = 10000
REWARD_BUFFER = np.empty(NUM_EPISODE)
AVG_REWARD_BUFFER = np.empty(NUM_EPISODE)
for episode_i in range(NUM_EPISODE):
    state, _ = env.reset()
    done = False
    episode_reward = 0

    while not done:
        action = agent.get_action(state)
        next_state, reward, done, _, _ = env.step(action)
        agent.update_q_table(state, action, reward, next_state, done)
        state = next_state
        episode_reward += reward

    REWARD_BUFFER[episode_i] = episode_reward
    AVG_REWARD_BUFFER[episode_i] = np.mean(REWARD_BUFFER[:episode_i+1])
    if (episode_i + 1) % PRINT_INTERVAL == 0:
        print(f"Episode : {episode_i + 1},Avg.Reward: {AVG_REWARD_BUFFER[episode_i]:.3f}")

plt.figure(figsize=(10,5))
plt.plot(np.arange(len(AVG_REWARD_BUFFER)), gaussian_filter1d(AVG_REWARD_BUFFER, sigma = 11),
             color='purple',linewidth = 1,label = 'Avg.Reward')
plt.title('Training Reward')
plt.xlabel('Episode')
plt.ylabel('Averange Reward')
plt.grid(True)
plt.savefig(current_path + f'/training_reward_{timestamp}.png',dpi = 300)
plt.show()
agent.plot_policy()

IS_TEST = input("Test the AI player? (y/n)")
if IS_TEST:
    RENDER = input("Render the game? (y/n)")
    env = gym.make(id = 'Blackjack-v1',render_mode = 'human')
    NUM_EPISODE_TEST   = 100
    total_reward = 0
    wins = 0
    losses = 0
    draws = 0

    print("\nStarting testing...")

    for episode_i in range(NUM_EPISODE_TEST):
        state, _ = env.reset()
        done = False
        episode_reward = 0

        while not done:
            state_key = agent.get_state_key
            action = np.argmax(agent.q_table[state_key])
            state, reward, done, _, _ =env.step(action)
            episode_reward += reward
            print(f"Episode:{episode_i}, State:{state}, Action{action}, Reward:{reward}")

            if RENDER.lower() == 'y':
                env.render
                time.sleep(1) #1 second

        total_reward += episode_reward
        if episode_reward > 0:
            wins += 1
        elif episode_reward < 0:
            losses += 1
        else:
            draws += 1

    print("\nTest Results:")
    print(f"Total episode: {NUM_EPISODE_TEST}")
    print(f"Avg. reward: {total_reward/NUM_EPISODE_TEST:.3f}")
    print(f"Win rate: {wins/NUM_EPISODE_TEST*100:1f}%")
    print(f"Loss rate: {losses/NUM_EPISODE_TEST*100:1f}%")
    print(f"Draw rate: {draws/NUM_EPISODE_TEST*100:1f}%")