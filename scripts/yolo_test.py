#!/usr/bin/env python3

import rospy
from ultralytics import YOLO
import cv2
import math
from single_query import SingleQueryModel
from std_msgs.msg import Float32


# Change this depending on system (i.e. which port webcam is in). Probably 0 or 1
CAMERA_NUM = 0
# start webcam


# model
model = YOLO("yolo-Weights/yolov8n.pt")  # v8x
classNames = ["Person"]

# reid
reid_model_name = "youtu"
reid_weights = "pretrained/youtu_reid_baseline_lite.onnx"
query_path = "queries/query_emerson.jpg"
sq_model = SingleQueryModel(reid_model_name, reid_weights, query_path)
# THRESHOLD = 0.02
# REID_RATE = 10  # controls how often to do REID. Once every REID_RATE iterations, so higher val is less often.


class HumanDetector:
    def __init__(self, threshold=0.02, rate=0.1):
        rospy.init_node("human_detector", anonymous=True)
        self.rate = rospy.Rate(rate)
        self.threshold = threshold

        self.cap_width = 640
        self.cap_height = 480
        self.cap = cv2.VideoCapture(CAMERA_NUM)
        self.cap.set(3, self.cap_width)
        self.cap.set(4, self.cap_height)

        #Publisher
        self.human_pub = rospy.Publisher('/human_orientation', Float32, queue_size=10)

    def get_human_orientation(self, x1, x2):
        center_x = (x1 + x2)/2
        return center_x / self.cap_width

    def detect_human(self):
        while not rospy.is_shutdown():
            _, img = self.cap.read()
            results = model(img, stream=True, classes=0, verbose=False)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # convert to int values

                    subimage = img[y1:y2, x1:x2]
                    dist = sq_model.test_matrix(subimage)
                    if dist < self.threshold:
                        self.human_pub.publish(self.get_human_orientation(x1, x2))

            self.rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

        
# def main():
#     i = 0
#     while True:
#         i = i % REID_RATE
#         success, img = cap.read()
#         results = model(img, stream=True, classes=0, verbose=False)

#         # coordinates
#         for r in results:
#             boxes = r.boxes
#             if i==0:
#                 print("==========================")
#             for box in boxes:
#                 # bounding box
#                 x1, y1, x2, y2 = box.xyxy[0]
#                 x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # convert to int values

#                 # do reid
#                 if i == 0:
#                     subimage = img[y1:y2, x1:x2]
#                     dist = sq_model.test_matrix(subimage)
#                     print(dist)
#                     if dist < THRESHOLD:
#                         box_color = (0, 255, 0)

#                     else:
#                         box_color = (255, 0, 255)
#                 # put box in cam
#                 cv2.rectangle(img, (x1, y1), (x2, y2), box_color, 3)

#                 # object details
#                 org = [x1, y1+20]
#                 font = cv2.FONT_HERSHEY_SIMPLEX
#                 fontScale = 1
#                 color = (255, 0, 0)
#                 thickness = 2

#                 cv2.putText(img, f"Dist={dist:.3f}", org, font, fontScale, color, thickness)

#         cv2.imshow('Webcam', img)
#         if cv2.waitKey(1) == ord('q'):
#             break
#         i += 1

#     cap.release()
#     cv2.destroyAllWindows()


# if __name__ == "__main__":
    # main()
