from styx_msgs.msg import TrafficLight
from infer import LightDetectionAndClassification, Detection_Model
from detection import ImageWrap

class TLClassifier(object):
    def __init__(self):

        #Create an instance of traffic light neural network classifier
        self.ldac = LightDetectionAndClassification(load_frozen = True, detection_model = Detection_Model.RCNN_INCEPTION)
        
        pass
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
           
        #Infer color of the traffic light using the classifier
        self.box, self.predictions, self.predicted_class, self.predicted_label , self.annotated_img, self.traffic_light = self.ldac.infer(ImageWrap(image), annotate=True, desired_labels=["Red", "Green", "Yellow"])
        
        label_to_id_map = {"Green": 2, "Yellow": 1, "Red": 0, "Off": 4}
        self.COLORID = label_to_id_map[self.predicted_label]
        return TrafficLight.self.COLORID
