#This file must be run from this folder, or change the paths to the data

import matplotlib.pyplot as plt
import numpy as np

def max_value(path):
    data = []
    time = []

    #get data from file
    with open(path, "r") as file:
        for i, line in enumerate(file):
            if i>5:
                nums = line.split("\t")
                data.append(float(nums[1]))
                time.append(float(nums[2]))

    return max(data)

def main():
    num_of_trials = 5
    sizes = [70, 80, 90]
    currents = [50, 75, 100, 125, 150]
    fingers = ['A', 'B', 'C']
    colors = ['blue', 'orange', 'green']
    all_values = np.zeros((len(sizes), len(currents), num_of_trials * len(fingers)))
    for i, size in enumerate(sizes):
        plt.figure(i)
        for j, finger in enumerate(fingers):
            trial_stdevs = np.array([])
            trial_averages = np.array([])
            for k, current in enumerate(currents):
                trial = np.zeros(num_of_trials)
                for l in range(num_of_trials):
                    file = 'Trial2/' + str(size) + 'mm/' + str(current) + 'mA/Finger' + finger + '/' + str(l+1) +'.log'
                    trial[l] = max_value(file)
                trial_averages = np.append(trial_averages, [np.average(trial)])
                trial_stdevs = np.append(trial_stdevs, [np.std(trial)])
                all_values[i, k, j*5:j*5+5] = trial
            plt.plot(currents, trial_averages, color = colors[j])
            plt.fill_between(currents, trial_averages - trial_stdevs, trial_averages + trial_stdevs, color = colors[j], alpha=.1, label='_nolegend_')
            # three_finger_average[i, :] += np.array(trial_averages)
        #labels
        plt.legend(["Finger A", "Finger B", "Finger C"])
        plt.xlabel("Motor Current (mA)")
        plt.xticks(currents)
        plt.ylabel("Fingertip Force (N)")
        plt.title('Fingertip Force for ' + str(size) + 'mm Diameter Sphere')
    
    # print(all_values)
    #plot the three fingers together
    three_finger_average = np.zeros((len(sizes), len(currents)))
    three_finger_stdev = np.zeros((len(sizes), len(currents)))
    for i, value in enumerate(all_values):
        three_finger_stdev[i] = value.std(axis=1)
        three_finger_average[i] = value.mean(axis=1)

    plt.figure(len(sizes))
    size_legend = []
    for i, size in enumerate(sizes):
        plt.plot(currents, three_finger_average[i, :])
        plt.fill_between(currents, three_finger_average[i, :] - three_finger_stdev[i, :], three_finger_average[i, :] + three_finger_stdev[i, :], color = colors[i], alpha=.1, label='_nolegend_')
        size_legend.append(str(size) + 'mm diameter apple')
    plt.legend(size_legend)
    plt.xlabel("Motor Current (mA)")
    plt.xticks(currents) 
    plt.ylabel("Average Fingertip Force (N)")
    plt.title('Fingertip Force for Different Sized Apples')

    plt.show()


if __name__ == '__main__':
    main()
    
                    

