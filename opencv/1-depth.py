import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 

        # Topics from Astra Pro (ROS 2 version)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)

        self.depth_image = None

        self.get_logger().info("YOLO + Depth node started!")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def image_callback(self, msg):
        if self.depth_image is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Color image conversion error: {e}")
            return

        results = self.model(color_image, conf=0.5)[0]  # confidence threshold

        # Inside image_callback, loop over boxes:
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])  # Get confidence score

            # Get center point of the box
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # Check depth bounds
            if 0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]:
                depth = self.depth_image[cy, cx]

                # Convert mm to meters (if needed)
                if depth == 0 or np.isnan(depth):
                    distance = -1.0
                else:
                    distance = float(depth) / 1000.0

                # Draw detection box and label with confidence & distance
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                conf_pct = int(conf * 100)
                label_text = f"{label}: {conf_pct}% - {distance:.2f} m" if distance > 0 else f"{label}: {conf_pct}% - N/A"
                cv2.putText(color_image, label_text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                print(f"Detected: {label}, Confidence: {conf_pct}%, Distance: {distance:.2f} m")


        cv2.imshow("YOLO + Depth", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
