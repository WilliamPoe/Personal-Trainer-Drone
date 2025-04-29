#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class Tracker(Node):
    def __init__(self):
        super().__init__('tracking')

        self.camera_sub = self.create_subscription(Image, '/bebop/camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

        # Load YOLOv5 or YOLOv8 model - pretrained on COCO
        self.model = YOLO("yolov8n.pt")  # You can use yolov5s.pt or yolov8s.pt if you prefer

        self.tracker = None
        self.tracking = False

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            if not self.tracking:
                boxes = self.detect_people(cv_image)

                if boxes:
                    (x1, y1, x2, y2) = self.select_target(boxes)
                    bbox = (x1, y1, x2 - x1, y2 - y1)

                    self.tracker = cv2.TrackerKCF_create()
                    self.tracker.init(cv_image, bbox)
                    self.tracking = True
                    self.get_logger().info("Tracker initialized.")
            else:
                success, bbox = self.tracker.update(cv_image)
                if success:
                    (x, y, w_box, h_box) = [int(v) for v in bbox]
                    cx = x + w_box // 2
                    cy = y + h_box // 2

                    # Draw box + center point
                    cv2.rectangle(cv_image, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                    offset_x = cx - width // 2
                    offset_y = cy - height // 2
                    self.get_logger().info(f"Offset: x={offset_x}, y={offset_y}")

                else:
                    self.get_logger().info("Lost tracking. Waiting to re-initialize.")
                    self.tracking = False

            cv2.imshow('Tracking Feed', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def detect_people(self, frame):
        # Use YOLO to detect people in the frame
        results = self.model(frame)
        boxes = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # Class 0 = person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    boxes.append((x1, y1, x2, y2))
                    padding = 0.1
                    dx = int((x2 - x1) * padding)
                    dy = int((y2 - y1) * padding)

                    x1 = max(0, x1 - dx)
                    y1 = max(0, y1 - dy)
                    x2 = min(img_width, x2 + dx)
                    y2 = min(img_height, y2 + dy)
        return boxes

    def select_target(self, boxes):
        # Choose the largest person by area
        largest = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        return largest

def main(args=None):
    rclpy.init(args=args)

    track = Tracker()

    rclpy.spin(track)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    track.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
