#!/usr/bin/python3

import grpc
import proto.robot_pb2 as robot_pb2
import proto.robot_pb2_grpc as robot_pb2_grpc

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
import cv2
import numpy as np

from google.protobuf.empty_pb2 import Empty 

def get_robot_data():
    channel = grpc.insecure_channel('192.168.3.168:50051')
    stub = robot_pb2_grpc.RobotServiceStub(channel)
    response = stub.GetRobotData(Empty())
    return response
    
if __name__ == '__main__':
    data = get_robot_data()
    pose_data = data.pose
    img_data = data.image.image_bytes
    print("x: {}, y: {}, yaw: {}".format(pose_data.x, pose_data.y, pose_data.yaw))
    print("image size: {}".format(len(img_data)))

    nparr = np.frombuffer(img_data,np.byte)
    img = cv2.imdecode(nparr,cv2.IMREAD_ANYCOLOR)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

