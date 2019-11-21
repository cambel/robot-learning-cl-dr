import glob
import numpy as np
from plotter_utils import *

import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import colors
matplotlib.style.use('seaborn')


def cumulative_reward_per_step(rewards):
    total_rewards = [0]
    for eps_rw in rewards:
        for r in eps_rw:
            total_rewards.append(total_rewards[-1]+r)
    return np.array(total_rewards)


def sum_reward_per_eps(rewards, cumulative=False):
    reward_per_eps = [0]
    total = 0
    for rw in rewards:
        if cumulative:
            total += np.sum(rw)
            reward_per_eps.append(total)
        else:
            reward_per_eps.append(np.sum(rw))
    return reward_per_eps


def flatten(rewards):
    rewards_flattened = np.array([])
    for rw in rewards:
        rewards_flattened = np.append(rewards_flattened, rw)
    return rewards_flattened


def load_common_data(folder_names):
    combined_data = [load_data(folder) for folder in folder_names]
    return np.concatenate(combined_data)


def load_data(folder_name):
    parent_folder = '~/'
    filename = glob.glob(parent_folder + 'results/'+folder_name+'/state_*')
    if len(filename) > 0:
        return np.load(filename[0], allow_pickle=True)
    raise ValueError("File not found: %s" % folder_name)


def single_plot_detailed(folders, label='', color='C0', cumulative=False):
    all_data = []
    min_len = 10e8
    for f in folders:
        data = cumulative_reward_per_step(load_data(f)[:, 1])
        all_data.append(data)
        if len(data) < min_len:
            min_len = len(data)

    all_data = np.array(all_data)

    for i in range(len(all_data)):
        all_data[i] = all_data[i][:min_len].flatten()

    print(all_data[0].shape, all_data[1].shape)

    all_data = np.stack((all_data[0], all_data[1]))
    y = np.average(all_data, axis=0)
    y_std = np.std(all_data, axis=0)
    x = np.linspace(0, len(y), len(y))

    plt.plot(x, y, color=color, label=label)
    plt.fill_between(x, y-y_std, y+y_std, antialiased=True,
                     linewidth=0.5, alpha=0.1, facecolor=color)


def print_successful_episodes(data):
    reward_details = np.array(data[:, 2])
    successes = 0
    for episode_rewards in reward_details:
        if episode_rewards[-1][3] > 0:
            successes += 1
    print("num of successes", successes, 'out of', len(
        reward_details), '%', round(successes/len(reward_details), 2))


def single_plot(data, label='', color='C0', cumulative=False, debug=False):
    obs = np.array(data[:, 0])
    rewards = np.array(data[:, 1])
    reward_details = np.array(data[:, 2])
    print(label)
    print_successful_episodes(data)

    if debug:
        print(data.shape)
        print(np.array(obs[1]).shape)
        print(np.array(rewards[1]).shape)
        print(np.array(reward_details[1]).shape)

    rewards = np.delete(rewards, [0, 1])

    rgb = colors.colorConverter.to_rgb(color)
    rgb_new = make_rgb_transparent(rgb, (1, 1, 1), 0.2)

    if cumulative:
        y = cumulative_reward_per_step(rewards)
        # y = np.log(y)
        x = np.linspace(0, len(y), len(y))
    else:
        y = np.array(sum_reward_per_eps(rewards))
        x = np.linspace(0, len(y), len(y))
        plt.plot(x, y, color=color, alpha=0.1)
        y = numpy_ewma_vectorized(np.array(sum_reward_per_eps(rewards)), 10)
    plt.plot(x, y, color=color, label=label)


def multi_plot(folder_names, labels=None, cumulative=False):
    if not labels:
        labels = folder_names
    _colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7']
    for i, folder in enumerate(folder_names):
        if isinstance(folder, list):
            single_plot_detailed(
                folder, label=labels[i], color=_colors[i], cumulative=cumulative)
        else:
            single_plot(load_data(folder),
                        label=labels[i], color=_colors[i], cumulative=cumulative)


# Multiplot
# folder_names = [
#     ['folder1', 'folder2'], # combined (different trials of same method)
#     'folder3', # different method
# ]

# labels = None
# multi_plot(folder_names, labels=labels, cumulative=True)

plt.xlabel('Steps', size='xx-large')
plt.ylabel('Cumulative Reward', size='xx-large')
plt.legend(title='Method')
plt.show()
