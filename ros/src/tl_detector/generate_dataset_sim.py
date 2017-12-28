import os
import csv
import cv2
from os.path import isfile, join
import yaml

def read_labels_yaml(file_yaml):
    with open(file_yaml, 'r') as stream:
        data_loaded = yaml.load(stream)

    list = []
    for item in data_loaded:
        filename = item["filename"].replace("sim_data_capture/", "")
        list.extend([[filename, d["xmin"], d["ymin"], float(d["xmin"]) + float(d["x_width"]), float(d["ymin"]) + float(d["y_height"]), None, d["class"], "" ] for d in item["annotations"]])
    #for row in data_loaded:
    #    list.append(row)
    # i need to strip the extra white space from each string in the row
    return (list)

FILE = 0
X1 = 1
Y1 = 2
X2 = 3
Y2 = 4
OCCLUDED = 5
LABEL = 6
ATTRIBUTES = 7

def crop_and_resize(labels, origin, destRgb, destGray):
    n = 0
    print(destRgb)
    print(destGray)

    for l in labels:
        img = cv2.imread(origin+l[FILE])
        y1 = int(l[Y1])
        y2 = int(l[Y2])
        x1 = int(l[X1])
        x2 = int(l[X2])

        roiRgb = img[y1:y2, x1:x2]
        roiGray = cv2.cvtColor(roiRgb, cv2.COLOR_BGR2GRAY)

        nameRgb = destRgb + l[ATTRIBUTES] + "\\" + str(n) + ".jpg"
        nameGray = destGray + l[ATTRIBUTES] + "\\" + str(n) + ".jpg"
        #print(nameRgb)
        #print(nameGray)
        cv2.imwrite(nameRgb, roiRgb)
        cv2.imwrite(nameGray, roiGray)
        n += 1

        if n%100 == 0:
            print("Image", n, "ouf ot", len(labels))

base = "C:\\datasets\\Udacity\\object-dataset-sim\\sim_training_data\\"
origin = base + "sim_data_capture"
#files = files_from(origin, include_sub_directories=False)
#files = [f for f in files if f.endswith(".jpg")]
labels = read_labels_yaml(base + "sim_data_annotations.yaml")

print(labels)

labels_lights = labels

print("Total files:", len(labels))
print("Frames annotated with Traffic lights:", len(labels_lights))

crop_and_resize(labels_lights, origin + "\\", origin + "\\outRgbSim", origin + "\\outGraySim")

