import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import numpy as np

class EyeViewer(Node):
    def __init__(self):
        super().__init__('Sim_IncisionPoints')   # initalize Node
        self.declare_parameter('updateSeconds', 5)  # set update rate for timer

        pos1 = np.array([0.747, 0.0, 0.25])        # position of incsion points
        pos2 = np.array([1.0,1.0,0.0])
        
        orient1 = np.array([0.0, 0.65, 0.0, 0.76])      # orientation of incision points (x,y,z,w)
        orient2 = np.array([1.0,0.0,0.0,0.0])

        # create markers and publishers for all parts of the eye model 
        self.insition_point1_marker, self.point1_pub = self.create_marker('sim/cut_model/incision_marker1', pos1, orient1)
        self.insition_point2_marker, self.point2_pub = self.create_marker('sim/cut_model/incision_marker2', pos2, orient2)

        self.timer = self.create_timer(self.get_parameter('updateSeconds').value, self.timer_callback)      # create timer 

     # timer may be removed -> one initial publish at beginning -> rviz has to be started before
    def timer_callback(self):

        self.point1_pub.publish(self.insition_point1_marker)
        self.point2_pub.publish(self.insition_point2_marker)



    def create_marker(self, marker_name, pos, orient):
            '''
            create publisher and marker for given marker name 
            '''
            
            # create publisher for topic Marker with name marker_name
            pub = self.create_publisher(Marker, marker_name, 10)

            marker = Marker()                   # create object to define Marker
            marker.header.frame_id = 'eye'      # set frame ID which is later linked to 'map'
            marker.id = 0
            marker.type = Marker.CYLINDER  
            marker.action = Marker.ADD
            
            # set visualization characteristics
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.001
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]

            marker.pose.orientation.x = orient[0]
            marker.pose.orientation.y = orient[1]
            marker.pose.orientation.z = orient[2]
            marker.pose.orientation.w = orient[3]

            return marker, pub
        

def main(args=None):
    rclpy.init(args=args)
    node = EyeViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
