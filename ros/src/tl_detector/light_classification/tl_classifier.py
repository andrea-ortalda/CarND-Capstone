from styx_msgs.msg import TrafficLight
import os
import cv2
import numpy as np
import tensorflow as tf
from keras.models import load_model
import rospy

class TLClassifier(object):

    def __init__(self):

        # Variables for traffic light detection
        self.sess = None
        self.image_tensor = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.num_detections = None

        # Model for traffic light detection
        self.current_path = os.path.dirname(os.path.realpath('Traffic_light_detector/'))
        self.model_detection_path = self.current_path + "/light_classification/Traffic_light_detector/frozen_inference_graph.pb"

        # Load the Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with open(self.model_detection_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name ='')

            self.sess = tf.Session(graph = self.detection_graph)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        # Model for traffic light classification
        self.model_classifier_path = self.current_path + '/light_classification/Traffic_light_detector/model_python2.h5'
        self.classifier = load_model(self.model_classifier_path)
        self.class_graph = tf.get_default_graph()

        # Variables for traffic light classification
        self.classification = None

    '''
    This method detects the traffic light state in the input camera image
    '''
    def get_classification(self, image):
        traffic_light_box = self.detect(image)
        if traffic_light_box is not None:
            #rospy.loginfo('TL Detector: Traffic light box found.')
            self.classify(traffic_light_box)
            if self.classification is not None:
                if self.classification == 0:
                    #rospy.loginfo('TL Classifier: Red')
                    return TrafficLight.RED
                elif self.classification == 1:
                    #rospy.loginfo('TL Classifier: Yellow')
                    return TrafficLight.YELLOW
                elif self.classification == 2:
                    #rospy.loginfo('TL Classifier: Green')
                    return TrafficLight.GREEN
                else:
                    #rospy.loginfo('TL Classifier: Unknown')
                    return TrafficLight.UNKNOWN
        else:
            #rospy.loginfo('TL Detector: Traffic light box not found.')
            return TrafficLight.UNKNOWN
    '''
    This method detects a traffic light box in an image and returns it
    '''
    def detect(self, image):
        traffic_light_first_box = None
        image_bgr = image
        image_expanded = np.expand_dims(image_bgr, axis=0)
        (boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores,
        self.detection_classes, self.num_detections],feed_dict ={self.image_tensor: image_expanded})
        for parameter in zip(boxes[0], classes[0], scores[0]):
         if parameter[1] == 10 and parameter[2] >= .5:
            box = parameter[0]
            x_min = int(box[0] * image_bgr.shape[0])
            x_max = int(box[2] * image_bgr.shape[0])
            y_min = int(box[1] * image_bgr.shape[1])
            y_max = int(box[3] * image_bgr.shape[1])
            if (x_max - x_min) > 10:
                image_box = image_bgr[x_min:x_max,y_min:y_max,:]
                if image_box.shape[0] != 0:
                    try:
                        traffic_light_first_box = cv2.resize(image_box,(14,32))
                        traffic_light_first_box = cv2.cvtColor(traffic_light_first_box, cv2.COLOR_BGR2RGB)
                        break
                    except:
                        pass
        return traffic_light_first_box

    '''
    This method classifies the traffic light state from a traffic light box image
    '''
    def classify(self, image):
        with self.class_graph.as_default():
            self.classification = np.argmax(self.classifier.predict(image.reshape(1,32,14,3)))
        return None
