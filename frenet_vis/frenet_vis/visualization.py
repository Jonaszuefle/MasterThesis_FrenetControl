import numpy as np
import rclpy
from rclpy.node import Node

from omni_msgs.msg import HIDState, FrenetState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utility_functions.path_utilities import ReferencePath
from utility_functions.constraint_utility import ConstraintDefinition

class FrenetVis(Node):

    def __init__(self):
        super().__init__('global2frenet')
        
        self.get_param()
        self.initialize_publisher_and_subscriber()
        self.initalize_reference_path()
        self.initialize_markers()
        self.create_timer(2, self.trajectory_callback)       # guarantees that the reference path is visible in rviz
        self.get_logger().info("Visualization Node has been initialized")

    
    def get_param(self):
        self.declare_parameter('reference_tracking.num_points', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.trajectory_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.trajectory_scaling_factor', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.enable_obstacle', rclpy.Parameter.Type.BOOL)
        self.num_points = self.get_parameter('reference_tracking.num_points').get_parameter_value().integer_value
        self.traj_scaling = self.get_parameter('reference_tracking.trajectory_scaling_factor').get_parameter_value().double_value
        self.trajectory_type = self.get_parameter('reference_tracking.trajectory_type').get_parameter_value().integer_value
        self.enable_obstacle = self.get_parameter('reference_tracking.enable_obstacle').get_parameter_value().bool_value


    def initialize_publisher_and_subscriber(self):
        self.ref_path_publisher = self.create_publisher(Marker, '/frenet/ref_path_marker', 1)
        self.state_marker_publisher = self.create_publisher(Marker, '/frenet/state_marker', 1)
        self.frenet_t_publisher = self.create_publisher(Marker, '/frenet/frenet_marker_t', 1)
        self.frenet_n_publisher = self.create_publisher(Marker, '/frenet/frenet_marker_n', 1)
        self.frenet_b_publisher = self.create_publisher(Marker, '/frenet/frenet_marker_b', 1)

        self.global_state_subscriber = self.create_subscription(HIDState, '/master/state', self.global_state_callback, 1)
        self.frenet_matrix_subscriber = self.create_subscription(FrenetState, '/master/frenet_state', self.frenet_state_callback, 1)


    def initalize_reference_path(self):
        '''
        Initalize the reference path, which consists of a set of points . A predefined path can be set in the yaml file.
        The path object is defined in the utility_functions.path_utilities module.
        '''
        self.path_obj_vis = ReferencePath(self.num_points, self.trajectory_type, trajectory_scaling= self.traj_scaling)        
        

    def initialize_markers(self):
        # Position Marker
        self.marker_pose = Marker()
        self.marker_pose.header.frame_id = 'map'
        self.marker_pose.type = Marker.SPHERE
        self.marker_pose.action = Marker.ADD

        self.marker_pose.scale.x = 0.02
        self.marker_pose.scale.y = 0.02
        self.marker_pose.scale.z = 0.02

        self.marker_pose.color.r = 1.0
        self.marker_pose.color.g = 0.0
        self.marker_pose.color.b = 0.0
        self.marker_pose.color.a = 1.0

        # Trajectory Marker
        self.marker_trajectory = Marker()
        self.marker_trajectory.header.frame_id = 'map'
        self.marker_trajectory.type = Marker.LINE_STRIP
        self.marker_trajectory.action = Marker.ADD

        self.marker_trajectory.scale.x = 0.002  # thickness
        self.marker_trajectory.color.a = 1.0  # transparency
        self.marker_trajectory.color.r = 1.0  
        self.marker_trajectory.color.g = 0.0  
        self.marker_trajectory.color.b = 0.0  

        for i in range(self.path_obj_vis.traj_matrix[1,:].shape[0]):
            p = Point()
            p.x = self.path_obj_vis.traj_matrix[1, i]
            p.y = self.path_obj_vis.traj_matrix[2, i]
            p.z = self.path_obj_vis.traj_matrix[3, i]
            self.marker_trajectory.points.append(p)

        self.initalize_frenet_matrix_markers()

        if self.enable_obstacle:
            self.initalize_obstacle_marker()
            self.get_logger().info("Obstacle marker has been initialized")


    def initalize_frenet_matrix_markers(self):
        '''
        Initalize the markers for the Frenet coordinate system.
        '''
        self.marker_t = Marker()
        self.marker_n = Marker()
        self.marker_b = Marker()

        self.create_frenet_frame_marker(self.marker_t)
        self.create_frenet_frame_marker(self.marker_n)
        self.create_frenet_frame_marker(self.marker_b)

        self.marker_t.color.r = 1.0     # set colours
        self.marker_n.color.g = 1.0
        self.marker_b.color.b = 1.0


    def create_frenet_frame_marker(self, marker):

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.color 
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.scale.x = 0.002
        marker.scale.y = 0.006
        marker.scale.z = 0.002


    def initalize_obstacle_marker(self):
        '''
        Initalize the obstacle marker
        '''
        constraint_obj = ConstraintDefinition(self.path_obj_vis, np.ones([8,]), np.ones([3,]), enable_obstacle=self.enable_obstacle)
        obstacle_fun = constraint_obj.obstacle_fun

        s_vals = np.linspace(0, self.path_obj_vis.arc_length, 100)
        obstacle_vals = obstacle_fun(s_vals)
        non_zero_idx = np.where(obstacle_vals > 0)[0]

        s_obstacle = s_vals[non_zero_idx]

        idx_obstacle = np.zeros(s_obstacle.shape[0], dtype=int)
        for i in range(len(s_obstacle)):
            idx_obstacle[i] = int(self.path_obj_vis.get_idx_from_arc_length(s_obstacle[i]))

        pos_obstacle = self.path_obj_vis.traj_matrix[1:4, idx_obstacle]

        self.marker_obstacle = Marker()
        self.marker_obstacle.header.frame_id = 'map'
        self.marker_obstacle.type = Marker.SPHERE_LIST
        self.marker_obstacle.action = Marker.ADD

        self.marker_obstacle.scale.x = 0.01
        self.marker_obstacle.scale.y = 0.01
        self.marker_obstacle.scale.z = 0.01

        self.marker_obstacle.color.r = 0.0
        self.marker_obstacle.color.g = 1.0
        self.marker_obstacle.color.b = 0.0
        self.marker_obstacle.color.a = 1.0

        for i in range(pos_obstacle[1,:].shape[0]):
            p = Point()
            p.x = pos_obstacle[0, i]
            p.y = pos_obstacle[1, i]
            p.z = pos_obstacle[2, i]
            self.marker_obstacle.points.append(p)

        self.create_timer(2, self.obstacle_callback)       # guarantees that the reference path is visible in rviz
        self.obstacle_publisher = self.create_publisher(Marker, '/frenet/obstacle_marker', 1)
        

    # ======= callbacks ========
    def global_state_callback(self, msg):
        '''
        Callback function for the global state subscriber
        '''

        self.marker_pose.header.stamp = self.get_clock().now().to_msg()
        self.marker_pose.pose = msg.pose
        self.state_marker_publisher.publish(self.marker_pose)

    
    def frenet_state_callback(self, msg):
        '''
        Callback function for the frenet state subscriber
        '''
        frenet_matrix = msg.frenet_matrix                   # frenet matrix as 9x1 array
        orth_pose = msg.orthogonal_projection_position

        scaling_factor = 0.035
        self.T = frenet_matrix[0:3] * scaling_factor        # scale arrays in order to have smaller arrows
        self.N = frenet_matrix[3:6] * scaling_factor
        self.B = frenet_matrix[6:9] * scaling_factor
 
        

        self.marker_t.points = [
        Point(x=orth_pose[0], y=orth_pose[1], z=orth_pose[2]),
        Point(x=orth_pose[0] + self.T[0], y=orth_pose[1] + self.T[1], z=orth_pose[2] + self.T[2])
        ]        

        self.marker_n.points = [
        Point(x=orth_pose[0], y=orth_pose[1], z=orth_pose[2]),
        Point(x=orth_pose[0] + self.N[0], y=orth_pose[1] + self.N[1], z=orth_pose[2] + self.N[2])
        ]

        self.marker_b.points = [
        Point(x=orth_pose[0], y=orth_pose[1], z=orth_pose[2]),
        Point(x=orth_pose[0] + self.B[0], y=orth_pose[1] + self.B[1], z=orth_pose[2] + self.B[2])
        ]

        self.frenet_t_publisher.publish(self.marker_t)
        self.frenet_n_publisher.publish(self.marker_n)
        self.frenet_b_publisher.publish(self.marker_b)
    

    def trajectory_callback(self):
        '''
        Callback function for the trajectory publisher -- triggered by slow timer
        '''
        self.ref_path_publisher.publish(self.marker_trajectory)


    def obstacle_callback(self):
        '''
        Callback function for the obstacle publisher -- triggered by slow timer
        '''
        
        self.obstacle_publisher.publish(self.marker_obstacle)
        
    

def main(args=None):
    rclpy.init(args=args)
    node = FrenetVis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()