import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import HIDState
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct

class CutModel_new(Node):
    def __init__(self):
        super().__init__('cut_model_node')

        self.pose_sub = self.create_subscription(HIDState, '/slave/state', self.pose_callback, 10)  # create tool pose subscriber

        # ---- Cut model ----
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'sim/cut_model/cut_points', 10)  # create publisher for cut point cloud

        #Initialize a PointCloud2 message
        self.point_cloud_msg = PointCloud2()
        self.point_cloud_msg.header.frame_id = 'eye'
        self.point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define fields
        self.point_cloud_msg.height = 1
        self.point_cloud_msg.width = 0
        self.point_cloud_msg.is_dense = True
        self.point_cloud_msg.is_bigendian = False

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        self.point_cloud_msg.fields = fields

        self.point_cloud_msg.point_step = 12  # 3 * 4 bytes
        self.point_cloud_msg.row_step = 0 

        self.data_length = 0       # define variable for length of data

        self.point_cloud_msg.data = bytearray()
        

        # ---- Scalpel ----
        self.scalpel_marker_pub = self.create_publisher(Marker, 'sim/cut_model/scalpel_marker', 10)  # create publisher for scalpel marker
        self.scalpel_orient_pub = self.create_publisher(PoseStamped, 'sim/cut_model/scalpel_axes', 10)  # create publisher for scalpel marker
        self.blade_orient_pub = self.create_publisher(PoseStamped, 'sim/cut_model/blade_axes', 10)  # create publisher for scalpel marker
        
        self.scalpel_marker = Marker()  # create topic of type Marker 
        self.scalpel_marker.id = 0
        self.scalpel_marker.type = Marker.MESH_RESOURCE  # define marker to use added mesh
        self.scalpel_marker.action = Marker.ADD  # create or update marker in rviz

        # calculate the cutting points along scalpel edge -> 33 points
        self.cutting_points = self.blade_model(15)
        
        # scalpel model is too large -> has to be scaled down heavily
        self.scalpel_marker.scale.x = 0.05
        self.scalpel_marker.scale.y = 0.05
        self.scalpel_marker.scale.z = 0.05
        
        self.scalpel_marker.color.r = 1.0
        self.scalpel_marker.color.g = 0.0
        self.scalpel_marker.color.b = 0.0
        self.scalpel_marker.color.a = 1.0
        
        self.scalpel_marker.mesh_resource = 'package://rviz_vis_model/models/scalpel_cataract_v4.obj'  # add path to 3D Model

        self.scalpel_orient = PoseStamped()

        self.blade_orient = PoseStamped()


    def pose_callback(self, msg:HIDState):
        '''
        Create new points if the cutting tool is inside the eye - published points model the cut into the eye
        Visualize the scalpel according to tool_pose

        Parameters
        ----------
        msg : PoseStamped
            Message received through subscriber
        '''

        # Update scalpel marker position and orientation
        self.scalpel_marker.pose = msg.tool_tip_pose
        self.scalpel_marker.header=msg.header

        self.scalpel_orient.pose = msg.pose
        self.scalpel_orient.header = msg.header

        self.blade_orient.pose = msg.tool_tip_pose
        self.blade_orient.header = msg.header




        self.scalpel_marker_pub.publish(self.scalpel_marker)  # publish marker
        self.scalpel_orient_pub.publish(self.scalpel_orient)
        self.blade_orient_pub.publish(self.blade_orient)

        # Calculate points on blade
        blade_points = self.calculate_blade_vertices(msg.tool_tip_pose, self.cutting_points)

        # Check if any part of the blade is within the eye
        for point in blade_points:
            if self.is_within_eye(point[0], point[1], point[2]):
                self.add_point(point[0], point[1], point[2])
                self.pointcloud_pub.publish(self.point_cloud_msg)


    def is_within_eye(self, x, y, z):
        '''
        Check whether cutting tool is inside the eye which is modeled using a semicircle

        Parameters
        ----------
        x,y,z : float
            Position of cutting tool
        '''
        center_x = 0.0
        center_y = 0.0
        center_z = 0.02
        radius = 0.77
        distance_squared = (x - center_x) ** 2 + (y - center_y) ** 2 + (z - center_z) ** 2
        
        return (distance_squared <= radius ** 2) and z > 0.0


    def add_point(self, x, y, z):
        '''
        Add new point to PointCloud2 message at the position x,y,z

        Parameters
        ----------
        x,y,z : float
            Position of cutting tool
        '''
        point = [x, y, z]
        self.point_cloud_msg.data.extend(struct.pack('fff', *point))        # add new points 
        self.data_length += 1          # iterate id

        # Update width and row_step
        self.point_cloud_msg.width = self.data_length      # adapt width of data
        self.point_cloud_msg.row_step = self.point_cloud_msg.point_step * self.point_cloud_msg.width


    def calculate_blade_vertices(self, pose, blade_local_vertices):
        '''
        Calculate the points of the blade based on the current pose

        Parameters
        ----------
        pose : geometry_msgs/Pose
            The pose of the scalpel

        Returns
        -------
        List of blade vertices as numpy arrays
        '''
        rotation = R.from_quat([            # Create a rotation object from the orientation
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        # Apply rotation and translation to the local vertices
        blade_global_vertices = rotation.apply(blade_local_vertices) + np.array([pose.position.x, pose.position.y, pose.position.z])

        return blade_global_vertices
    

    def blade_model(self, num_points):
        '''
        Define the blade model with interpolation between vertices
        
        Parameters
        ----------
        num_points : 
            number of points between tip and one vertice

        Returns
        -------
        (num_points * 2 + 3) points on th scalpel edge 
        '''
        # Define the local vertices of the blade
        tip = np.array([0.0, 0.0, 0.0])
        base_left = np.array([0.0, 0.051, 0.087])
        base_right = np.array([0.0, -0.051, 0.087])

        # Interpolate points between the tip and the base vertices
        blade_local_vertices = [tip]
        for i in range(1, num_points + 1):
            t = i / (num_points + 1)
            interp_left = tip * (1 - t) + base_left * t
            interp_right = tip * (1 - t) + base_right * t
            blade_local_vertices.append(interp_left)
            blade_local_vertices.append(interp_right)

        return np.array(blade_local_vertices)


def main(args=None):
    rclpy.init(args=args)
    node = CutModel_new()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
