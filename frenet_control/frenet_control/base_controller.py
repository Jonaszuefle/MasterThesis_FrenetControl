import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np

from threading import Thread

from omni_msgs.msg import HIDFeedback, HIDState, FrenetState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3

import time

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

class BaseController(Node):
    '''
    Base Class to provide the basic functionalities for the controller, which all controllers have in commen.
    Initalizes the publishers and subscribers and provides the basic structure for the control loop.
    
    Input:
        - Frenet Transformation [FrenetState.msg]
    Output:
        - Control Output [HIDFeedback.msg]
    '''

    def __init__(self, name):
        super().__init__(name)
        
        self.get_params()
        if not self.use_fake_hardware:
            self.init_tf2_listener()

        self.initialize_publisher_and_subscriber()
        self.initalize_state_vectors()
        self.get_logger().info("Base Control Node has been initialized")

        


    def get_params(self):
        self.declare_parameter('system_parameters.system_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('system_parameters.system_parameters', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('system_parameters.system_type', rclpy.Parameter.Type.INTEGER)

        self.declare_parameter('control_parameters.control_period', rclpy.Parameter.Type.DOUBLE)
        
        self.declare_parameter('reference_tracking.trajectory_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.num_points', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('reference_tracking.trajectory_scaling_factor', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.tangent_velocity', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.d_t_ref', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.d_n_ref', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('reference_tracking.d_b_ref', rclpy.Parameter.Type.DOUBLE)
        
        self.declare_parameter('human_control', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('fake_hardware', rclpy.Parameter.Type.BOOL)

        # define parent class variables 
        self.system_period = self.get_parameter('system_parameters.system_period').get_parameter_value().double_value
        self.system_parameters = self.get_parameter('system_parameters.system_parameters').get_parameter_value().double_array_value
        self.system_type = self.get_parameter('system_parameters.system_type').get_parameter_value().integer_value
        
        self.control_period = self.get_parameter('control_parameters.control_period').get_parameter_value().double_value

        self.trajectory_type = self.get_parameter('reference_tracking.trajectory_type').get_parameter_value().integer_value
        self.num_points = self.get_parameter('reference_tracking.num_points').get_parameter_value().integer_value
        self.traj_scaling = self.get_parameter('reference_tracking.trajectory_scaling_factor').get_parameter_value().double_value
        self.tangent_velocity = self.get_parameter('reference_tracking.tangent_velocity').get_parameter_value().double_value
        self.d_t_ref = self.get_parameter('reference_tracking.d_t_ref').get_parameter_value().double_value
        self.d_n_ref = self.get_parameter('reference_tracking.d_n_ref').get_parameter_value().double_value
        self.d_b_ref = self.get_parameter('reference_tracking.d_b_ref').get_parameter_value().double_value
        
        self.human_control = self.get_parameter('human_control').get_parameter_value().bool_value
        self.use_fake_hardware = self.get_parameter('fake_hardware').get_parameter_value().bool_value
            

    def initialize_publisher_and_subscriber(self):
        self.publisher_group = MutuallyExclusiveCallbackGroup()                 # callback group for the publisher 
        self.frenet_state_callback_group = MutuallyExclusiveCallbackGroup()     # callback group for the subscriber (control calculation)   -- run each group in parallel

        self.frenet_state_subscriber = self.create_subscription(FrenetState, '/master/frenet_state', self.frenet_state_callback, 1, callback_group=self.frenet_state_callback_group)

        if not self.human_control:
            self.force_feedback_publisher = self.create_publisher(HIDFeedback, '/master/feedback', 1)
        else:
            self.force_feedback_publisher_human = self.create_publisher(HIDFeedback, '/master/feedback_human', 1)

        self.create_timer(self.control_period, self.publish_control_output, callback_group=self.publisher_group)

        self.force_feedback_kuka_pub = self.create_publisher(Vector3, 'frenet/force_feedback',1)


    def initalize_state_vectors(self):
        self.new_control_output = False           # flag for new control output
        self.first_state_received = False         # flag for first global state received

        self.frenet_state_reference = np.zeros((6, 1))
        self.frenet_state_reference[0] = self.d_t_ref
        self.frenet_state_reference[1] = self.d_n_ref
        self.frenet_state_reference[2] = self.d_b_ref
        self.frenet_state_reference[3] = self.tangent_velocity

        self.control_output_delay_count = 0
        self.control_outputs = np.zeros([3, 10])    # calculated control outputs -- placeholder


    def frenet_state_callback(self, msg):
        '''
        Callback function for the frame state subscriber.
        Calculate the control output based on the selected control method.
        '''
        if self.first_state_received == False:
            self.first_state_received = True

        t_start = time.time()

        self.frenet_state_vector = np.array([msg.frenet_state]).reshape(-1,1)
        self.frenet_matrix = np.array([msg.frenet_matrix]).reshape(3,3, order='F')
        self.point_on_ref = np.array([msg.orthogonal_projection_position]).reshape(-1,1)
        self.s_dot = msg.s_dot
        self.s_prog = msg.s_progress

        self.control_outputs = self.control_step(self.frenet_state_vector, self.frenet_matrix)

        t_end = time.time()

        self.duration_control = t_end - t_start


    def control_step(self):
        '''
        Placeholder for the control_step function. This function is implemented in the child classes -> MPC or PI controller.
        '''
        raise NotImplementedError("control_step function must be implemented in the child class")


    def publish_control_output(self):
        '''
        Callback function for publishing the control output.
        This function is called periodically based on the control period defined in the parameters.
        If the computation time of the control_step function is longer than the control period, the next predicted control output is used.

        Inputs:
            - self.control_outputs: the control outputs calculated in the control_step function [3, N] where N is the prediciton horizon
        Output:
            - Publishes the control output to the force_feedback_publisher
        '''

        if self.first_state_received == False:
            pass

        else:
            if self.new_control_output == True:
                self.new_control_output = False

                control_output = self.control_outputs[:,0]
                self.control_output_delay_count = 1         # start the delay counter

            else:
                control_output = self.control_outputs[:,self.control_output_delay_count]
                self.control_output_delay_count += 1

                #self.get_logger().info(f'Control Step calc took too long, count: {self.control_output_delay_count}')

                if self.control_output_delay_count == len(self.control_outputs[1,:])-1:
                    self.control_output_delay_count -= 2

            if np.isnan(control_output).any():
                control_output = np.zeros(3)

            msg_feedback = HIDFeedback()
            msg_feedback.desired_force.x = control_output[0]
            msg_feedback.desired_force.y = control_output[1]
            msg_feedback.desired_force.z = control_output[2] 

    
            if not self.human_control:
                self.force_feedback_publisher.publish(msg_feedback)
            else:
                self.force_feedback_publisher_human.publish(msg_feedback)
            
            if not self.use_fake_hardware:
                msg_force_feedback = Vector3()

                kuka_base_frame = control_output
                
                kuka_tool_frame = self.rot_kuka.apply(kuka_base_frame)
                msg_force_feedback.x = kuka_tool_frame[0] * 10
                msg_force_feedback.y = kuka_tool_frame[1] * 10
                msg_force_feedback.z = kuka_tool_frame[2] * 10

                self.force_feedback_kuka_pub.publish(msg_force_feedback)

    def init_tf2_listener(self):

        # TF Buffer + Listener (nimmt automatisch TF-Daten aus /tf entgegen)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rot_kuka = R.from_quat([0, 0, 0, 1])

        # Timer: wir fragen regelmäßig eine TF ab
        self.timer = self.create_timer(0.05, self.lookup_transform)

    def lookup_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'tool0',        # target frame
                'iiwa_base',       # source frame
                Time()          # "latest available"
            )

            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]

            self.rot_kuka = R.from_quat(quat)


        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            
            
    

def main(args=None):
    rclpy.init(args=args)
    node = BaseController('base_controller')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # executor_thread = Thread(target=node.spin, args=(executor,), daemon=True)
    # executor_thread.start()

    executor.spin()

    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()