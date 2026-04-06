import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.model = YOLO('yolov8n.pt')  # downloads on first run if not cached
        self.get_logger().info('YOLOv8n model loaded.')

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            10,
        )
        self.pub = self.create_publisher(
            CompressedImage,
            '/yolo/image_raw/compressed',
            10,
        )

    def _image_callback(self, msg: CompressedImage):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode image.')
            return

        # Run inference
        results = self.model(frame, conf=0.5, verbose=False)

        # Draw bounding boxes
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = f'{self.model.names[cls_id]} {conf:.2f}'

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                )

        # Re-encode and publish
        success, encoded = cv2.imencode('.jpg', frame)
        if not success:
            self.get_logger().warn('Failed to encode annotated image.')
            return

        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = 'jpeg'
        out_msg.data = encoded.tobytes()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
