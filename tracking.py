#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from flight import DroneController

class Tracker(Node):
    def __init__(self):
        super().__init__('tracking')

        self.camera_sub = self.create_subscription(Image, '/bebop/camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

        self.model = YOLO("yolov8n.pt")

        self.tracker = None
        self.tracking = False

        self.Drone_Controller = DroneController()

        # For recording
        self.video_writer = None
        self.record_path = os.path.expanduser("~/tracking_output.mp4")
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 30  # You can adjust this based on your actual stream

        self.video_initialized = False

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            # Initialize video writer
            if not self.video_initialized:
                self.video_writer = cv2.VideoWriter(self.record_path, self.fourcc, self.fps, (width, height))
                self.video_initialized = True
                self.get_logger().info(f"Recording started: {self.record_path}")

            if not self.tracking:
                boxes = self.detect_people(cv_image, width, height)

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

                    cv2.rectangle(cv_image, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                    x_offset = cx - width // 2
                    y_offset = cy - height // 2
                    self.get_logger().info(f"Offset: x={x_offset}, y={y_offset}")

                    self.Drone_Controller.tracking(x_offset, y_offset)

                else:
                    self.get_logger().info("Lost tracking. Waiting to re-initialize.")
                    self.tracking = False

            # Record the frame
            if self.video_writer:
                self.video_writer.write(cv_image)

            cv2.imshow('Tracking Feed', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def detect_people(self, frame, img_width, img_height):
        results = self.model(frame)
        boxes = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    padding = 0.1
                    dx = int((x2 - x1) * padding)
                    dy = int((y2 - y1) * padding)

                    x1 = max(0, x1 - dx)
                    y1 = max(0, y1 - dy)
                    x2 = min(img_width, x2 + dx)
                    y2 = min(img_height, y2 + dy)

                    boxes.append((x1, y1, x2, y2))
        return boxes

    def select_target(self, boxes):
        return max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))

def main(args=None):
    rclpy.init(args=args)
    track = Tracker()
    try:
        rclpy.spin(track)
    finally:
        if track.video_writer:
            track.video_writer.release()
        track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
