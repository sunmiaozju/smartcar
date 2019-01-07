# !/usr/bin/python
# coding:utf-8
"""
Used to extract waypoints from csv file
  changge self.threshold
  change self.file
"""

import csv
import matplotlib.pyplot as plt
import numpy as np


class ExtractWaypoints():
    def __init__(self):
        self.file = "waypoints.csv"
        self.shift = 0
        self.data = []
        self.final = []
        self.pre = None
        self.threshold = 1.0

    def _initial(self):
        with open(self.file, 'r') as fr:
            for line in fr.readlines():
                self.data.append(map(float, line.strip().split(',')))
        self.final.append(self.data[0])
        self.pre = self.data[0]

    def _save(self):
        with open("filtered_waypoints.csv", 'w') as csvfile:
            writer = csv.writer(csvfile)
            for line in self.final:
                writer.writerow(line)

    def run(self):
        self._initial()
        for line in self.data:
            length = np.sqrt(np.power(line[0] - self.pre[0], 2) + np.power(line[1] - self.pre[1], 2))
            self.shift += length
            if self.shift >= self.threshold:
                self.final.append(line)
                self.shift = 0
            self.pre = line
        self._save()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return exc_type, exc_val, exc_tb


if __name__ == '__main__':
    with ExtractWaypoints() as app:
        app.run()
