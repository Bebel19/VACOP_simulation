import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data

class CameraDisplay(Node):
    def __init__(self):
        super().__init__('camera_display')
        

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw/image_color',
            self.listener_callback,
            qos_profile_sensor_data)
            
        self.bridge = CvBridge()
        

        self.window_name = "Vue Robot VACOP (Haute Definition)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 500)
        
        self.get_logger().info("Viewer démarré. Vous pouvez redimensionner la fenêtre.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()