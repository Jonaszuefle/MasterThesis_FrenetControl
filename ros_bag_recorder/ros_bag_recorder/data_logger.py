import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

import datetime
import os
import shutil
from ament_index_python.packages import get_package_share_directory

import rosbag2_py
from omni_msgs.msg import HIDState, FrenetState, HIDFeedback
from geometry_msgs.msg import WrenchStamped, Vector3

import rosbag2_py._storage 
        
        
class MotionRecorder(Node):
    def __init__(self):
        super().__init__('motion_recorder')

        self.get_param()
        self.init_subscriber()
        self.create_rosbag_writer()

        self.save_current_config_file()     # save the current config file into the bag folder

        self.get_logger().info('Motion Recorder Node has been started.')
        

    def get_param(self):
        self.declare_parameter('recording.file_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('recording.file_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('human_control', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('fake_hardware', rclpy.Parameter.Type.BOOL)

        self.file_path = self.get_parameter('recording.file_path').value
        self.file_name = self.get_parameter('recording.file_name').value
        self.human_control_enable = self.get_parameter('human_control').value
        self.use_fake_hardware = self.get_parameter('fake_hardware').value
    
    def init_subscriber(self):
        # global state
        self.state_subscriber = self.create_subscription(HIDState, '/master/state', self.state_callback, 1)                                     # global state

        # frenet state
        self.frenet_state_subscriber = self.create_subscription(FrenetState, '/master/frenet_state', self.frenet_state_callback, 1)             # frenet state
        self.controll_feedback_subscriber = self.create_subscription(HIDFeedback, '/master/feedback', self.control_feedback_callback, 1)        # feedback automation

        # kuka specific
        if not self.use_fake_hardware:
            self.control_feedback_kuka_sub = self.create_subscription(Vector3, 'frenet/force_feedback', self.control_feedback_kuka_callback, 1)
            self.human_force_sub = self.create_subscription(WrenchStamped, '/human_force_base_frame', self.human_force_callback, 1)

        # simulated human
        if self.human_control_enable:
            self.frenet_state_subscriber_human = self.create_subscription(FrenetState, '/master/frenet_state_human', self.human_frenet_state_callback, 1)
            self.control_feedback_subscriber_human = self.create_subscription(HIDFeedback, '/master/feedback_human', self.human_feedback_callback, 1)
            
        

    def create_rosbag_writer(self):
        self.writer = rosbag2_py.SequentialWriter()
        file_date_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')        
        self.declare_parameter('Bagfile', self.file_path+file_date_str+'-'+self.file_name+".bag")        
        self.bagfile = self.get_parameter('Bagfile').value

        self.get_logger().info(f"Recording to {self.bagfile}")

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.bagfile,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/state', type='omni_msgs/msg/HIDState', serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/frenet_state', type='omni_msgs/msg/FrenetState', serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/feedback', type='omni_msgs/msg/HIDFeedback', serialization_format='cdr'))

        if not self.use_fake_hardware:
            self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/frenet/force_feedback', type='geometry_msgs/msg/Vector3', serialization_format='cdr'))
            self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/human_force_base_frame', type='geometry_msgs/msg/WrenchStamped', serialization_format='cdr'))

        if self.human_control_enable:
            self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/feedback_human', type='omni_msgs/msg/HIDFeedback', serialization_format='cdr'))
            self.writer.create_topic(rosbag2_py._storage.TopicMetadata(name='/master/frenet_state_human', type='omni_msgs/msg/FrenetState', serialization_format='cdr'))


    def write_to_rosbag(self, topic: str, msg):
        self.writer.write(topic, serialize_message(msg), self.get_clock().now().nanoseconds)


    def save_current_config_file(self):
        package_name = 'frenet_system_bringup'
        config_file = 'config/params.yaml'
        
        config_file_path = os.path.join(get_package_share_directory(package_name), config_file)
        dest_path = os.path.join(self.bagfile, 'params.yaml')

        shutil.copy(config_file_path, dest_path)


    def state_callback(self, msg):
        self.write_to_rosbag('/master/state', msg)

    def control_feedback_callback(self, msg):
        self.write_to_rosbag('/master/feedback', msg)

    def frenet_state_callback(self, msg):
        self.write_to_rosbag('/master/frenet_state', msg)

    def human_feedback_callback(self, msg):
        self.write_to_rosbag('/master/feedback_human', msg)

    def human_frenet_state_callback(self, msg):
        self.write_to_rosbag('/master/frenet_state_human', msg)

    def control_feedback_kuka_callback(self, msg):
        self.write_to_rosbag('/frenet/force_feedback', msg)

    def human_force_callback(self, msg):
        self.write_to_rosbag('/human_force_base_frame', msg)
    
               

   

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