#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
from ultralytics import YOLO

# Suppress YOLO model logging
logging.getLogger("ultralytics").setLevel(logging.WARNING)


class ImageProcessing(Node):
    def __init__(self):
        super().__init__('imageprocessing')
        self.image_subscriber = self.create_subscription(Image, '/right_depth_camera/image_raw', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.classes_to_detect = ["boat", "person", "car", "truck", "bicycle"]
        self.colors = {                 
            "boat":     (0, 0, 255),    # Red
            "person":   (255, 0, 0),    # Blue
            "car":      (0, 255, 0),    # Green
            "truck":    (204, 0, 102),  # Pink
            "bicycle":  (0, 255, 255),  # Yellow
        }
        self.classes = []
        self.model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)
        for id, name in self.model.names.items():
            if name in self.classes_to_detect:
                self.classes.append(id)
        self.br = CvBridge()
        self.image_subscriber
        
    def image_callback(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg)
        ret = True
        if ret:
            self.results = self.model.predict(self.frame, conf = 0.4, iou = 0.2, classes = self.classes)
            # self.results = self.model(self.frame) # return a list of Results objects

            ## Process results list
            for result in self.results:
                boxes = result.boxes  # Boxes object for bounding box outputs
                # masks = result.masks  # Masks object for segmentation masks outputs
                # keypoints = result.keypoints  # Keypoints object for pose outputs
                # probs = result.probs  # Probs object for classification outputs
                # obb = result.obb  # Oriented boxes object for OBB outputs
                # result.show()
                # result.save(filename="result.jpg")  # save to disk
                
                for box in boxes:
                    x, y, w, h = box.xywh[0][0:4]
                    class_id = int(box.cls[0]) # Get the class id of the detected object
                    confidence = box.conf[0]
                    class_name = self.model.names[class_id]
                    if class_name in self.colors:
                        color = self.colors[class_name]
                        yolo_box = int(x-w/2), int(y-h/2), int(w), int(h)
                        # self.get_logger().info(f"{class_name} : {yolo_box[0]}, {yolo_box[1]}, {yolo_box[0] + yolo_box[2]}, {yolo_box[1] + yolo_box[3]}\n")
                        cv2.rectangle(self.frame,(yolo_box[0],yolo_box[1]), (yolo_box[0] + yolo_box[2], yolo_box[1] + yolo_box[3]),color,6)

                        label = f"{class_name} {confidence*100:.1f}%"
                        (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                        
                        label_background_top_left = (yolo_box[0], yolo_box[1] - label_height - baseline)
                        label_background_bottom_right = (yolo_box[0] + label_width, yolo_box[1])
                        cv2.rectangle(self.frame, label_background_top_left, label_background_bottom_right, color, thickness=cv2.FILLED)
                        
                        cv2.putText(self.frame, label, (yolo_box[0], yolo_box[1] - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            self.image_publisher.publish(self.br.cv2_to_imgmsg(self.frame))
                    
                    
def main(args=None):
    
    rclpy.init(args=args)
    image_process = ImageProcessing()
    rclpy.spin(image_process)
    image_process.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()