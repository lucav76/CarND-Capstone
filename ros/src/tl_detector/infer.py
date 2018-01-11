import sys
import tensorflow as tf
import numpy as np
import os
from detection import LightDetection
from detection import ImageWrap
from detection import freeze_session
import net
import cv2
import utils
from PIL import Image
from keras.models import load_model
from keras import backend as K1
from keras.layers.core import K as K2
import time
import os
import os.path
from collections import OrderedDict

print("Python: " + sys.version)
print("TensorFlow:" + tf.__version__)

K1.set_learning_phase(0)
K2.set_learning_phase(0)

SSD_MOBILE_NET = "ssd_mob_frozen_inference_graph.pb"
SSD_INCEPTION = "ssd_inc_frozen_inference_graph.pb"
RCNN_INCEPTION = "rcnn_inc_inference_graph.pb"
RCNN_RESNET_101 = "rcnn_res101_frozen_inference_graph.pb"


class LightDetectionAndClassification:
    def __init__(self, load_frozen = True, detection_model = SSD_MOBILE_NET):
        print("Detection Frozen Graph File:  - " + str(detection_model))

        self.det = LightDetection(detection_model)
        self.det.load_graph()
        self.classifier_net = net.LightNet(None, False)
        #self.classifier_model = self.classifier_net.create_model()

        use_frozen = False

        if load_frozen:
            print("Loading frozen graph")
            self.load_model_graph('model-prod-frozen.pb')
        else:
            print("Loading Keras graph")
            self.classifier_model = load_model('model-prod-ck.h5')
            print("Model:" + str(self.classifier_model))



    def freeze(self):
        # Freeze
        print("Freezing the graph...")
        print("Output: " + str(self.classifier_model.output.op.name))
        print("Input: " + str(self.classifier_model.input.op.name))

        frozen_graph = freeze_session(K1.get_session(), output_names=[self.classifier_model.output.op.name])

        print("Saving the graph in TF...")
        from tensorflow.python.framework import graph_io
        graph_io.write_graph(frozen_graph, ".", "model_main.pb", as_text=False)
        print("Done...")

    def load_model_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.classifier_detection_graph = graph

        # The classification of the object (integer id).
        self.detection_classes = self.classifier_detection_graph.get_tensor_by_name('fcout/Softmax:0')

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.classifier_detection_graph.get_tensor_by_name('Normalize_input:0')

        self.sess = tf.Session(graph=self.classifier_detection_graph)

    def area(self, box):
        y1, x1, y2, x2 = box
        return (x2 - x1) * (y2 - y1)

    def find_biggest(self, boxes):
        max_size = 0
        biggest = None

        for box in boxes:
            size = self.area(box)

            if (size>max_size):
                max_size = size
                biggest = box

        return biggest

    def annotate_image(self, img, box, label):
        x1, y1, x2, y2 = box
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, label, (x1, y2+30), font, 1, (0, 255, 0))
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 1)


    def current_milli_time(self):
        return int(round(time.time() * 1000))

    def convert_box(self, box):
        y1, x1, y2, x2 = box
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        return (x1, y1, x2, y2)

    #box, predictions, predicted_class, predicted_label, img, traffic_light
    def find_best_box(self, image_wrap, boxes_ascending, desired_labels):
        while len(boxes_ascending)>0:
            biggest = boxes_ascending.pop()
            box = self.convert_box(biggest)

            img = image_wrap.get_image_bgr()
            traffic_light = img[box[1]:box[3], box[0]:box[2]]
            traffic_light = cv2.cvtColor(traffic_light, cv2.COLOR_BGR2GRAY);
            traffic_light_64_64 = cv2.resize(traffic_light, (64, 64), interpolation=cv2.INTER_CUBIC)
            traffic_light_64_64 = utils.resize_to_1_if_required(traffic_light_64_64)

            predictions = self.predict(np.array([traffic_light_64_64]))
            print("Raw predictions: " + str(predictions))
            predicted_class = np.argmax(predictions)
            predicted_label = self.classifier_net.data_labes[predicted_class]

            if (desired_labels is None or predicted_label in desired_labels):
                print("+++ Accepting size:" + str(self.area(box)) + " - Prediction: " + predicted_label)

                return box, predictions, predicted_class, predicted_label, img, traffic_light

            print("--- Skipping size:" + str(self.area(box)) + " - Prediction: " + predicted_label)

        return None, None, None, None, None, None

    def predict(self, img):
        classes = self.sess.run(self.detection_classes, feed_dict={self.image_tensor: img})

        return classes

    def infer(self, image_wrap, annotate, desired_labels = None, resize = True, confidence_cutoff = 0.6):
        RESIZE_WIDTH = 300

        start = self.current_milli_time()
        original_width, original_height = image_wrap.get_size()

        if resize and original_width>RESIZE_WIDTH:
            image_wrap.resize_crop_or_pad_horizontal(RESIZE_WIDTH)
            # print("Resize to " + str(image_wrap.get_size()))

        boxes = self.det.infer(image_wrap, confidence_cutoff=confidence_cutoff)
        t1 = self.current_milli_time()

        if len(boxes) == 0:
            return None, None, None, None, None, None

        print("Boxes: ", boxes)
        boxes_ascending = sorted(boxes, key=lambda box: self.area(box), reverse=False)
        print("Boxes Ascending: ", boxes_ascending)

        box, predictions, predicted_class, predicted_label, img, traffic_light = self.find_best_box(image_wrap, boxes_ascending, desired_labels)

        if annotate:
            annotated_image = None if img is None else self.annotate_image(img, box, predicted_label)

        t2 = self.current_milli_time()

        print("Timing: ", (t1-start), (t2-t1))

        if resize and original_width>RESIZE_WIDTH and box is not None:
            factor = float(original_width) / RESIZE_WIDTH
            x1, y1, x2, y2 = box
            box = (int(x1*factor), int(y1*factor), int(x2*factor), int(y2*factor))

        return box, predictions, predicted_class, predicted_label , annotated_image if annotate else img, traffic_light



    def infer_and_save(self, image_file, desired_labels = None, resize = True, confidence_cutoff = 0.6):
        print(image_file)
        image_wrap = ImageWrap(cv2.imread("assets" + os.sep + image_file), True)

        biggest_box, predictions, prediction_class, prediction_label, annotated, img_box = self.infer(image_wrap, True, desired_labels=desired_labels, resize=resize, confidence_cutoff = confidence_cutoff)

        print("Predictions:" + str(predictions))
        print("Biggest box: " + str(biggest_box) + " - " + str(prediction_class) + " - " + str(prediction_label))

        image_wrap.save("out_infer" + os.sep  + image_file)
        cv2.imwrite("out_infer" + os.sep + "roi_" + image_file, img_box)

    def infer_and_save_dir(self, path, desired_labels = None, resize = True, confidence_cutoff = 0.6):
        files = utils.files_only(path)

        for file in files:
            if not file.endswith(".jpg"):
                continue
            image_wrap = ImageWrap(cv2.imread(file), True)

            print("File: " + str(file))
            biggest_box, predictions, prediction_class, prediction_label, annotated, img_box = self.infer(image_wrap, True, desired_labels=desired_labels, resize=resize, confidence_cutoff = confidence_cutoff)

            if prediction_label is None:
                continue

            #print("Predictions:" + str(predictions))
            #print("Biggest box: " + str(biggest_box) + " - " + str(prediction_class) + " - " + str(prediction_label))

            file_name = file.replace(path, "")
            file_name = file_name.replace(os.sep , "")

            print(file_name)
            dir = path + os.sep + prediction_label
            if not os.path.isdir(dir):
                os.mkdir(dir)
            #image_wrap.save(dir + os.sep  + file_name)
            cv2.imwrite(dir + os.sep  + file_name, img_box)
            #cv2.imwrite("out_infer" + os.sep + "roi_" + image_file, img_box)



