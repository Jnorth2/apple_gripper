#This visualizes data from Trial1 of force data. This is now out of date.
#This file must be run from this folder, or change the paths to the data

import matplotlib.pyplot as plt
import numpy as np

def locate_index_of_deltas_v2(data, intercept=0.5):
    """
    Useful to locate in square signals, the index where the signal intersects a certain value
    """

    POIS_plus = []
    POIS_minus = []

    previous = data[0]

    for i in range(len(data)):

        if (previous < intercept) and (data[i] > intercept):
            POIS_plus.append(i)     # collects all points with positive gradient
        if (previous > intercept) and (data[i] < intercept):
            POIS_minus.append(i)    # collects all points with negative gradient

        previous = data[i]

    # print('For the intercept: ', intercept)
    # print('POIs of positive deltas: ', POIS_plus)
    # print('POIs of negative deltas: ', POIS_minus)

    return POIS_plus, POIS_minus

def max_values(path):
    max_vals = []
    data = []
    time = []

    #get data from file
    with open(path, "r") as file:
        for i, line in enumerate(file):
            if i>5:
                nums = line.split("\t")
                data.append(float(nums[1]))
                time.append(float(nums[2]))

    #find max locations
    max_ortho = max(data)
    pois_pos, pois_neg = locate_index_of_deltas_v2(data, max_ortho*0.45)
    cycles = len(pois_pos)
    # print('Number of cycles: ', cycles)

    #find max values
    for i in range(cycles):
        start = pois_pos[i]
        end = pois_neg[i]
        max_vals.append(max(data[start: end]))

    return max_vals

def main():
    #loop through all the files
    sizes = [70, 80, 90]
    currents = [50, 75, 100, 125, 150]
    fingers = ['A', 'B', 'C']
    colors = ['blue', 'orange', 'green']
    three_finger_average = np.zeros((len(sizes), len(currents)))
    for i, size in enumerate(sizes):
        plt.figure(i)
        for j, finger in enumerate(fingers):
            average_max_vals = []
            confidence_mins = []
            confidence_maxs = []
            for current in currents:
                file = 'Trial1/' + str(size) + 'mm/' + str(current) + 'mA_finger' + finger + '.log'
                #get max values as well as ranges of values
                max_vals = max_values(file)
                average_max_vals.append(sum(max_vals)/10)
                confidence_mins.append(min(max_vals))
                confidence_maxs.append(max(max_vals))
            #plot
            plt.plot(currents, average_max_vals, color = colors[j])
            plt.fill_between(currents, confidence_mins, confidence_maxs, color = colors[j], alpha=.1, label='_nolegend_')
            
            three_finger_average[i, :] += np.array(average_max_vals)
        #labels
        plt.legend(["Finger A", "Finger B", "Finger C"])
        plt.xlabel("Motor Current (mA)")
        plt.xticks(currents)
        plt.ylabel("Fingertip Force (N)")
        plt.title('Fingertip Force for ' + str(size) + 'mm Diameter Sphere')

    
    plt.figure(len(sizes))
    
    three_finger_average /= len(fingers)
    size_legend = []
    for i, size in enumerate(sizes):
        plt.plot(currents, three_finger_average[i, :])
        size_legend.append(str(size) + 'mm diameter apple')
    plt.legend(size_legend)
    plt.xlabel("Motor Current (mA)")
    plt.xticks(currents) 
    plt.ylabel("Average Fingertip Force (N)")
    plt.title('Fingertip Force for Different Sized Apples')

    plt.show()



if __name__ == '__main__':
    main()
    # data = []
    # time = []
    # with open('90mm/150mA_fingerA.log', "r") as file:
    #     for i, line in enumerate(file):
    #         if i>5:
    #             nums = line.split("\t")
    #             data.append(float(nums[1]))
    #             time.append(float(nums[2]))
    # max_ortho = max(data)
    # pois_pos, pois_neg = locate_index_of_deltas_v2(data, max_ortho*0.45)
    # cycles = len(pois_pos)
    # # print('Number of cycles: ', cycles)
    
    # plt.plot(time, data)
    # plt.show()
