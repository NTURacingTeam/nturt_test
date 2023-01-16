#!/usr/bin/env python3

# general import
import csv
from glob import glob
import matplotlib.pyplot as plt
import numpy as np
import statistics

# ros2 import
from ament_index_python.packages import get_package_share_directory

class PlotTestResult:
    # output text color
    __bold_red = "\033[1;31m"
    __bold_yellow = "\033[1;33m"
    __bode_white = "\033[0;1m"
    __white = "\033[0m"

    def __init__(self):
        self.__file_name, self.__frame_count, self.__latency = self.__load_csv_file()
        self.__plot_result()

    def __load_csv_file(self):
        # get all csv files in "nturt_rpi_can_latency_test" package shared directory
        csv_files = glob(get_package_share_directory("nturt_rpi_can_latency_test") + '/*.csv')
        if len(csv_files) < 1:
            print(f"{self.__bold_red}Error: {self.__bode_white}No data yet available. {self.__white}Please launch the test node first to create one.")
            exit()
        elif len(csv_files) > 1:
            print(f"{self.__bold_yellow}Warn: {self.__bode_white}Multiple data exists. {self.__white}Choosing most recent one.")
            csv_files.sort(reverse=True)

        file_name = csv_files[0]

        print(f"Loding data from: {file_name}")
        csv_file = open(file_name, 'r')
        csv_reader = csv.reader(csv_file)
        
        # skip headers
        csv_reader.__next__()

        frame_count = []
        latency = []
        for row in csv_reader:
            frame_count.append(int(row[0]))
            latency.append(1000 * float(row[1]))
        
        return file_name, frame_count, latency

    def __plot_result(self):
        # statistics
        mean = statistics.mean(self.__latency)
        stdev = statistics.stdev(self.__latency)
        max = np.max(self.__latency)

        # configure data to histogram
        counts, bins = np.histogram(self.__latency, bins=100)

        # plotting
        plt.hist(bins[:-1], bins, weights=counts, log=True, color="lightblue",
            label=f"Latency, with mean: {round(mean, 3)}, stdev: {round(stdev, 3)}, max: {round(max, 3)} [ms]")
        plt.title("RPI CAN Latency Round Trip Test")
        plt.xlabel("Time [ms]")
        plt.ylabel("Count")
        plt.grid()
        plt.legend()

        #saving figure
        figure_name = self.__file_name.rstrip(".csv") + ".png"
        print(f"Figure saved as: {figure_name}")
        plt.savefig(figure_name)
        plt.show()

if __name__ == "__main__":
    plot_test_result = PlotTestResult()
