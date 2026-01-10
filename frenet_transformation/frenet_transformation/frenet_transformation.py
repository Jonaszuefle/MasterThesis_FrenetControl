import rclpy
from rclpy.node import Node
import numpy as np

from omni_msgs.msg import HIDState, FrenetState

from utility_functions.path_utilities import ReferencePath
from utility_functions.transformation_utilities import GeometricTransformation

import time

class FrenetTransformation(Node):
    '''
    Node for the Frenet Transformation. This node subscribes to the global state of the device and calculates the Frenet state vector and the Frenet matrix.
    '''

    def __init__(self, name):
        super().__init__(name)
        
        self.get_params()
        self.initialize_publisher_and_subscriber()
        self.initalize_geometric_transformation()
        self.initalize_frenet_state_vector()
        
        self.get_logger().info("Base Control Node has been initialized")


    def get_params(self):
        self.declare_parameter('system_parameters.system_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.trajectory_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.frame_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.num_points', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.trajectory_scaling_factor', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('geometric_transformation.d_t_tresh', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('geometric_transformation.kappa_tresh', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('geometric_transformation.num_points_segment', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('geometric_transformation.segment_factor', rclpy.Parameter.Type.DOUBLE)
       
        self.system_period = self.get_parameter('system_parameters.system_period').get_parameter_value().double_value
        self.trajectory_type = self.get_parameter('reference_tracking.trajectory_type').get_parameter_value().integer_value
        self.frame_type = self.get_parameter('reference_tracking.frame_type').get_parameter_value().integer_value
        self.num_points = self.get_parameter('reference_tracking.num_points').get_parameter_value().integer_value
        self.traj_scaling = self.get_parameter('reference_tracking.trajectory_scaling_factor').get_parameter_value().double_value

        self.d_t_tresh = self.get_parameter('geometric_transformation.d_t_tresh').get_parameter_value().double_value
        self.kappa_tresh = self.get_parameter('geometric_transformation.kappa_tresh').get_parameter_value().double_value
        self.num_points_seg = self.get_parameter('geometric_transformation.num_points_segment').get_parameter_value().integer_value
        self.seg_factor = self.get_parameter('geometric_transformation.segment_factor').get_parameter_value().double_value
       
            

    def initialize_publisher_and_subscriber(self):

        self.global_state_subscriber = self.create_subscription(HIDState, '/master/state', self.global_state_callback, 1)

        self.frenet_state_publisher = self.create_publisher(FrenetState, '/master/frenet_state', 1)


    def initalize_geometric_transformation(self):
        '''
        Initalize the geometric transformation class. This class provides the methods to transform the global state to the frenet.
        '''
        self.path_obj = ReferencePath(num_points=self.num_points, trajectory_type=self.trajectory_type, num_interp_points=self.num_points_seg, segment_factor=self.seg_factor, kappa_treshhold=self.kappa_tresh, comp_type=0, trajectory_scaling=self.traj_scaling, frame_type=self.frame_type)
        self.transf_obj = GeometricTransformation(self.path_obj, self.system_period, self.d_t_tresh)


    def initalize_frenet_state_vector(self):

        self.frenet_state_vector = np.zeros((6, 1), dtype=float)
        self.frenet_matrix = np.zeros((3, 3), dtype=float)


    def global_state_callback(self, msg):
        '''
        Callback function for the cartesian state subscriber.
        Calculate the Frenet Transformation: Frenet state vector and the Frenet matrix.
        '''


        t_start = time.time()

        global_state_vector = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.velocity.x, msg.velocity.y, msg.velocity.z])
 
        self.frenet_state_vector, self.frenet_matrix, self.point_on_ref, self.s_dot, self.s_prog = self.transf_obj.global_to_frenet(global_state_vector)

        t_end = time.time()

        self.duration_control = t_end - t_start

        self.publish_frenet_state()


    def publish_frenet_state(self):
        '''
        Publish the Frenet state meassage.
        '''

        msg_frenet = FrenetState()                                          # published for evaluation purpose
        msg_frenet.frenet_state = self.frenet_state_vector
        msg_frenet.frenet_matrix = self.frenet_matrix.flatten('F')          # flatten the matrix using column major
        msg_frenet.orthogonal_projection_position = self.point_on_ref

        msg_frenet.s_dot = self.s_dot
        msg_frenet.s_progress = self.s_prog

        self.frenet_state_publisher.publish(msg_frenet)
        
            
    

def main(args=None):
    rclpy.init(args=args)
    node = FrenetTransformation('frenet_transformation')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()