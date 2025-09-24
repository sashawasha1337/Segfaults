##each robot will need json credentials,
##the json file should be in the same directory as firebase publishing script


from platform import node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
from std_msgs.msg import String

def ros_time_to_seconds(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9

def detection_to_dict(detection):
    
    result = {
        "center": {
            "x": detection.bbox.center.x,
            "y": detection.bbox.center.y
        },
        "size": {
            "x": detection.bbox.size.x,
            "y": detection.bbox.size.y
        },
        "results": []
    }
    for res in detection.results:
        result["results"].append({
            "id": res.id,
            "score": res.score,
            "hypothesis": {
                "class_id": res.hypothesis.class_id,
                "class_name": res.hypothesis.class_name
            }
        })
    return result


class EventCompilerNode(Node):
    def __init__(self):
        super().__init__('event_compiler_node')
        self.subscription1 = Subscriber(self, Image, '/camera/image_raw')
        ##this subscribes to example gps topic, can change this to whatever it actually is 
        ##self.subscription2 = Subscriber(self, NavSatFix, '/gps/fix')
        self.subscription3 = Subscriber(self, Detection2DArray, 'detection_events')

        ##ensures that the time of the bounding box message and image are close
        self.ts = ApproximateTimeSynchronizer([self.subscription1, self.subscription3], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)
        self.bridge = CvBridge()
        ##publish compiled events to compiled_events topic
        self.publisher_ = self.create_publisher(Detection2DArray, 'compiled_events', 10)
        self.get_logger().info("Event Compiler node initialized")

    def synced_callback(self, image_msg:Image, det_msg:Detection2DArray):
        # Convert ROS Image msg to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        # Here you can process the frame and detection data into a json which will be published as an event

        event={
            "time": ros_time_to_seconds(image_msg.header.stamp),
            "frame_id": image_msg.header.frame_id,
            "detections": len(det_msg.detections),
            "location": "testLoc",
            "robotID": "testRobot"
        }
        self.get_logger().info(f"EVENT: {event}  ")
        self.publisher_.publish(String(data=json.dumps(event)))


def main(args=None):
    rclpy.init(args=args)
    node = EventCompilerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


