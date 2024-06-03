#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

import proto.robot_pb2 as robot_pb2
import proto.robot_pb2_grpc as robot_pb2_grpc
import grpc
from concurrent import futures

'''
message Pose {
    double x = 1;
    double y = 2;
    double yaw = 3;
}

message ImageData {
    bytes image_bytes = 1;
}



'''

class RobotServiceServicer(robot_pb2_grpc.RobotServiceServicer):
    def __init__(self):
        self.bridge = CvBridge()
        self.image = robot_pb2.ImageData()
        self.pose = robot_pb2.Pose()

    def GetRobotData(self, request, context):
        return robot_pb2.RobotData(pose=self.pose,image=self.image)

    def image_callback(self, msg):
        #self.publisher.publish(detections_msg)
        #self.get_logger().info('Published pose from aruco {}'.format(2))
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        img_encoded = cv2.imencode('passthough', cv_image)[1].tobytes()
        image_data = image_pb2.ImageData(image_bytes=img_encoded)
        self.image_bytes = image_data

    def pose_callback(self,msg):
        
        self.pose.x = msg.pose.position.x
        self.pose.y = msg.pose.position.y
        orientation = msg.pose.orientation
        
        _, _, self.pose.yaw = self.quaternion_to_euler(orientation)


    @staticmethod
    def quaternion_to_euler(quat):
        # Convert quaternion to euler angles (roll, pitch, yaw)
        import math
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

class ROS2gRPCNode(Node):
    def __init__(self):
        super().__init__('ros2_grpc_node')
        self.srv = RobotServiceServicer()
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.srv.pose_callback,10)
        self.pose_sub

        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.srv.image_callback,10)
        self.image_sub




def main(args=None):
    rclpy.init(args=args)
    node = ROS2gRPCNode()

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_pb2_grpc.add_RobotServiceServicer_to_server(
            node.srv,server)
    server.add_insecure_port('[::]:50051')
    server.start()
    node.get_logger().info('gRPC server started on port 50051')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    server.stop()

if __name__ == '__main__':
    main()
