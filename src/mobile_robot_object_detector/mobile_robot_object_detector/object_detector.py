#!/home/fasta/PythonVEnvs/rosailab/bin/python3
"""
ROS 2 YOLO Object Detector Node

- Subscribes:  /camera/image_raw       (sensor_msgs/Image)
- Publishes:   camera/image_detected_objects (sensor_msgs/Image)  [annotated]

Parameters (set via ROS 2 params or `--ros-args -p key:=value`):
  - model      (str)   : YOLO model path/name, e.g. "yolo11n.pt" (default)
  - conf       (float) : Confidence threshold (default: 0.25)
  - imgsz      (int)   : Inference image size (default: 640)
  - device     (str)   : "cpu", "cuda:0" or "0" (default: auto-detected)
  - half       (bool)  : Use FP16 if on GPU (default: True)
  - max_det    (int)   : Max detections per image (default: 300)

Notes:
  * Requires: ultralytics, cv_bridge, OpenCV.
  * Efficient: non-blocking callback that drops frames if inference is busy.
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

try:
    import torch
    TORCH_AVAILABLE = True
except Exception:
    TORCH_AVAILABLE = False

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        # -------------------- Parameters --------------------
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("conf", 0.15)
        self.declare_parameter("imgsz", 640)
        # Auto-pick device if possible
        default_device = "cuda:0" if TORCH_AVAILABLE and torch.cuda.is_available() else "cpu"
        self.declare_parameter("device", default_device)
        self.declare_parameter("half", True)
        self.declare_parameter("max_det", 300)

        self.model_name: str = self.get_parameter("model").get_parameter_value().string_value
        self.conf: float = float(self.get_parameter("conf").get_parameter_value().double_value)
        self.imgsz: int = int(self.get_parameter("imgsz").get_parameter_value().integer_value or 640)
        self.device: str = self.get_parameter("device").get_parameter_value().string_value or default_device
        self.use_half: bool = bool(self.get_parameter("half").get_parameter_value().bool_value)
        self.max_det: int = int(self.get_parameter("max_det").get_parameter_value().integer_value or 300)

        self.get_logger().info(
            f"Loading YOLO model '{self.model_name}' on device '{self.device}' "
            f"(conf={self.conf}, imgsz={self.imgsz}, half={self.use_half}, max_det={self.max_det})"
        )

        # -------------------- YOLO Model --------------------
        try:
            self.model = YOLO(self.model_name)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model '{self.model_name}': {e}")
            raise

        # Move to device (optional but explicit)
        try:
            if TORCH_AVAILABLE:
                self.model.to(self.device)
        except Exception as e:
            self.get_logger().warn(f"Could not move model to device '{self.device}': {e}")

        # -------------------- ROS I/O -----------------------
        self.bridge = CvBridge()

        # Publisher for annotated images (use sensor-data QoS for realtime)
        self.pub_annot = self.create_publisher(
            Image, "camera/image_detected_objects", qos_profile_sensor_data
        )

        # Subscriber to raw camera images
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_cb, qos_profile_sensor_data
        )

        # Guard to avoid backlog: drop frame if we're still processing last one
        self._busy = False

        self.get_logger().info("YOLO detector node ready.")

    def image_cb(self, msg: Image) -> None:
        if self._busy:
            # Drop frame to keep latency low
            return
        self._busy = True

        try:
            # Convert ROS Image -> OpenCV BGR image
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            self._busy = False
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected conversion error: {e}")
            self._busy = False
            return

        try:
            # Run YOLO inference (Ultralytics handles BGR/RGB internally)
            results = self.model.predict(
                source=frame_bgr,
                conf=self.conf,
                imgsz=self.imgsz,
                device=self.device,
                half=(self.use_half and "cpu" not in self.device),
                max_det=self.max_det,
                verbose=False,
                stream=False,
            )

            # One image in, one result out
            res = results[0]

            # Render boxes + labels to a numpy array (BGR)
            annotated_bgr = res.plot()  # fast built-in visualizer

            # Convert back to ROS Image
            out_msg: Image = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
            # Preserve timing/frame info for sync
            out_msg.header = msg.header

            # Publish
            self.pub_annot.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Inference/publish error: {e}")
        finally:
            self._busy = False


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
