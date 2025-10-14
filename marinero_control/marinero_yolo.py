#!/usr/bin/env python3

import cv2
import time
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
from ultralytics import YOLO
from std_msgs.msg import Float64MultiArray, Bool

# Suppress YOLO model logging
logging.getLogger("ultralytics").setLevel(logging.WARNING)

class ImageProcessing(Node):
    def __init__(self):
        super().__init__("imageprocessing")
        self.image_subscriber = self.create_subscription(Image, "/right_depth_camera/image_raw", self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, "/camera/yolo_processed_image", 10)
        self.classes_to_detect = ["boat", "person", "car", "truck", "SMART", "SMART_FIRE"]
        self.colors = {     ## BGR ##               
            "boat":         (204, 0, 102),  # Purple
            "person":       (0, 50, 100),   # Brown
            "car":          (0, 255, 0),    # Green
            "SMART":        (255, 0, 0),    # Blue 
            "SMART_FIRE":   (0, 0, 255),    # Red 
        }
        self.classes = []
        self.boxes = []
        self.model = YOLO("yolov8s_marinaPunat.pt")

        # self.model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)
        for id, name in self.model.names.items():
            if name in self.classes_to_detect:
                self.classes.append(id)
        self.br = CvBridge()

        self.take_photo_subscriber = self.create_subscription(Bool, "/take_photo", self.take_photo_callback, 10)
        self.box_center_publisher = self.create_publisher(Float64MultiArray, "/yolo_box_center", 10)
        self.boat_box_center = (0.0, 0.0)
        self.image_center = (320.0, 240.0)
        self.take_photo = False


    def take_photo_callback(self, msg):
        self.take_photo = bool(msg.data)


    def image_callback(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg)
        ret = True
        if not ret:
            return

        self.results = self.model.predict(self.frame, conf = 0.6, iou = 0.2, classes = self.classes)
        # self.results = self.model(self.frame) # return a list of Results objects

        # Track the highest-confidence and highest box are of a boat class
        min_boat_area = 45000
        last_best_boat_center = (0.0, 0.0)

        boats = []
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
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]

                if class_name in self.colors:
                    # Draw bounding boxes for all detected objects
                    color = self.colors[class_name]
                    yolo_box = int(x - w / 2), int(y - h / 2), int(w), int(h)
                    cv2.rectangle(self.frame, (yolo_box[0], yolo_box[1]), (yolo_box[0] + yolo_box[2], yolo_box[1] + yolo_box[3]), color, 2)

                    label = f"{class_name} {confidence*100:.1f}%"
                    (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)
                    cv2.rectangle(self.frame, (yolo_box[0], yolo_box[1] - label_height - baseline), (yolo_box[0] + label_width, yolo_box[1]), color, thickness=cv2.FILLED)
                    cv2.putText(self.frame,label, (yolo_box[0], yolo_box[1] - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    if class_name == "boat":
                        boat_box_area = float(w * h)
                        if boat_box_area >= min_boat_area:
                            boats.append({
                                "center": [float(x), float(y)],
                                "confidence": confidence,
                                "area": boat_box_area
                            })
                            # last_best_boat_center = (x, y)
                            # Draw center of this detected boat
                            # cv2.circle(self.frame, (int(last_best_boat_center[0]), int(last_best_boat_center[1])), 8, (0, 255, 255), -1)

        # Publish boat center if needed
        if self.take_photo:
            # Pick the primary boat closest to image center
            if boats:
                boats.sort(key=lambda b: math.hypot(self.image_center[0]-b['center'][0], 
                                                    self.image_center[1]-b['center'][1]))
                primary_boat = boats[0]
                self.boat_box_center = primary_boat["center"]
                cv2.circle(self.frame, (int(self.boat_box_center[0]), int(self.boat_box_center[1])), 8, (0, 255, 255), -1)
            else:
                self.boat_box_center = (float("nan"), float("nan"))  # Explicit: no boat
            self.box_center_publisher.publish(Float64MultiArray(data=[self.boat_box_center[0], self.boat_box_center[1]]))
        else:
            boats = []
            time.sleep(1.0)

        # Publish the annotated image
        self.image_publisher.publish(self.br.cv2_to_imgmsg(self.frame))

    # def reference_frame(self):
    #     # Draw reference rectangle in the middle of the image
    #     ref_w, ref_h = 200, 200  # width * height
    #     ref_area = ref_w * ref_h
    #     center_x, center_y = self.frame.shape[1] // 2, self.frame.shape[0] // 2
    #     top_left = (center_x - ref_w // 2, center_y - ref_h // 2)
    #     bottom_right = (center_x + ref_w // 2, center_y + ref_h // 2)

    #     cv2.rectangle( self.frame, top_left, bottom_right, (0, 255, 255), 2)
    #     cv2.putText(self.frame, f"Ref. {ref_area} px^2", (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

def main(args=None):
    
    rclpy.init(args=args)
    image_process = ImageProcessing()
    rclpy.spin(image_process)
    image_process.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()