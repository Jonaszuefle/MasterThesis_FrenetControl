import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

import subprocess
import datetime
import numpy as np

import rosbag2_py
from omni_msgs.msg import HIDState, FrenetState
from geometry_msgs.msg import Vector3, WrenchStamped
import time


import os

import rosbag2_py._storage 
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_list = dir_path.split('/')
FILEPATH = '/'.join(dir_list[:dir_list.index('install')])+'/src/Rosbags/record_motion/'

        
        
class MotionRecorder(Node):
    def __init__(self):
        super().__init__('motion_recorder')
        

        # Subscribers
        self.state_subscriber = self.create_subscription(HIDState, '/master/state', self.state_callback, 1)
        self.control_feedback_subscriber = self.create_subscription(FrenetState, '/master/frenet_state', self.control_feedback_callback, 1)
        self.create_rosbag_writer()

        # KUKA
        self.control_feedback_kuka_sub = self.create_subscription(Vector3, 'frenet/force_feedback', self.control_feedback_kuka_callback, 1)
        self.human_force_sub = self.create_subscription(WrenchStamped, '/human_force_base_frame', self.human_force_callback(), 1)

        self.get_logger().info('Motion Recorder Node has been started.')


    def create_rosbag_writer(self):
        self.writer = rosbag2_py.SequentialWriter()
        file_date_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')        
        self.declare_parameter('Bagfile', FILEPATH+file_date_str+'-'+input("Filename: ")+".bag")        
        self.bagfile = self.get_parameter('Bagfile').value
        self.get_logger().info(f"Recording to {self.bagfile}")
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.bagfile,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/state', type='omni_msgs/msg/HIDState', serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/frenet_state', type='omni_msgs/msg/FrenetState', serialization_format='cdr'))

        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/frenet/force_feedback', type='geometry_msgs/msg/Vector3', serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/human_force_base_frame', type='geometry_msgs/msg/WrenchStamped', serialization_format='cdr'))


    def write_to_rosbag(self, topic: str, msg):
        self.writer.write(topic, serialize_message(msg), self.get_clock().now().nanoseconds)

    def control_feedback_kuka_callback(self, msg):
        self.write_to_rosbag('/frenet/force_feedback', msg)

    def human_force_callback(self, msg):
        self.write_to_rosbag('/human_force_base_frame', msg)

    def state_callback(self, msg):
        self.write_to_rosbag('/master/state', msg)

    def control_feedback_callback(self, msg):
        self.write_to_rosbag('/master/frenet_state', msg)       

   

def main(args=None):
    rclpy.init(args=args)
    node = MotionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #open_rosbag = input('Open rosbag in rqt_bag? y/n: ')
        #if open_rosbag in 'yY':
        #    subprocess.Popen(['rqt_bag',node.bagfile])
        node.destroy_node()

if __name__ == '__main__':
    main()