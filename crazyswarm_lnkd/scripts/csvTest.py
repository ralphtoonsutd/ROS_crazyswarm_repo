#!/usr/bin/env python3
import csv


if __name__ == "__main__":
    # read and write a csv here to see whats up
    with open('testCSV.csv', 'w+', newline='') as csvfile:
        newdata = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
        newdata.writerow(['lighthouse1']+[1.70]+[0.34]+[0.45])
        newdata.writerow(['lighthouse2']+[1.70]+[0.34]+[0.45])

    with open('testCSV.csv', newline='') as csvfile:
        csvfile = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in csvfile:
            print(row[0])
