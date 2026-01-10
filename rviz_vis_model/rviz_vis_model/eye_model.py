import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class EyeViewer(Node):
    def __init__(self):
        super().__init__('Sim_Eye_model')   # initalize Node
        self.declare_parameter('updateSeconds', 5)  # set update rate for timer

        # create markers and publishers for all parts of the eye model 
        self.cornea_marker, self.cornea_pub = self.create_marker('sim/tissue_model/cornea_marker', 'package://rviz_vis_model/models/Cornea.stl')
        self.iris_marker, self.iris_pub = self.create_marker('sim/tissue_model/iris_marker', 'package://rviz_vis_model/models/Iris.stl')
        self.Lense_marker, self.Lense_pub = self.create_marker('sim/tissue_model/lense_marker', 'package://rviz_vis_model/models/Lense.stl')
        self.sclera_marker, self.sclera_pub = self.create_marker('sim/tissue_model/sclera_marker', 'package://rviz_vis_model/models/Sclera.stl')

        self.timer = self.create_timer(self.get_parameter('updateSeconds').value, self.timer_callback)      # create timer 

     # timer may be removed -> one initial publish at beginning -> rviz has to be started before
    def timer_callback(self):
        self.cornea_marker.header.stamp = self.get_clock().now().to_msg()
        self.iris_marker.header.stamp = self.get_clock().now().to_msg()
        self.Lense_marker.header.stamp = self.get_clock().now().to_msg()
        self.sclera_marker.header.stamp = self.get_clock().now().to_msg()

        self.cornea_pub.publish(self.cornea_marker)
        self.iris_pub.publish(self.iris_marker)
        self.Lense_pub.publish(self.Lense_marker)
        self.sclera_pub.publish(self.sclera_marker)


    def create_marker(self, marker_name, mesh_data):
            '''
            create publisher and marker for given marker name and path to 3D model 
            add transparency to cornea model for better visualization
            '''
            
            # create publisher for topic Marker with name marker_name
            pub = self.create_publisher(Marker, marker_name, 10)

            marker = Marker()                   # create object to define Marker
            marker.header.frame_id = 'eye'      # set frame ID which is later linked to 'map'
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE  # use models for output
            marker.action = Marker.ADD
            
            # set visualization characteristics
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            # add transparency for Cornea
            if marker_name == 'sim/tissue_model/cornea_marker':
                marker.color.a = 0.7
            
            # link path to 3D-model
            marker.mesh_resource = mesh_data

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
