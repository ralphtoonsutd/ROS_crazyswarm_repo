#!/usr/bin/env python

import csv


if __name__ == "__main__":
    # read and write a csv here to see whats up

    # with open('testCSV.csv', 'w+', newline='') as csvfile:
    #     newdata = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
    #     newdata.writerow(['LHID']+['X']+['Y']+['Z'])
    #     newdata.writerow(['lighthouse1']+[1.70]+[0.34]+[0.45])
    #     newdata.writerow(['lighthouse2']+[1.70]+[0.34]+[0.45])

    # with open('testCSV.csv', newline='') as csvfile:
    #     csv_Reader = csv.DictReader(csvfile, delimiter=' ', quotechar='|')

    #     for row in csv_Reader:
    #         print(row['Formation'])

    coordDict = csv.DictReader(
        open('testCSV.csv', newline=''), delimiter=' ', quotechar='|')

    # Iterate over coord dictionary
    for row in coordDict:
        print(row['Formation'])

    lineCoords = [0]*10
    print(lineCoords)
