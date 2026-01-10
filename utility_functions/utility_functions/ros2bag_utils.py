# utils/rosbag_utils.py

import rosbag2_py
from omni_msgs.msg import HIDState, FrenetState, HIDFeedback
from geometry_msgs.msg import WrenchStamped, Vector3
import numpy as np
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import os
from scipy.interpolate import interp1d

class RosbagReader:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.reader.open(storage_options, converter_options)
        self.topic_types = self.reader.get_all_topics_and_types()
        self.type_map = {topic.name: topic.type for topic in self.topic_types}

        print(self.type_map)

    def read_ros_bag(self, topic_names):
        '''
        function to read the topics from the rosbag file

        Input:  
            - topic_name: list of topics to be read from the rosbag file
        '''
        # dictionary to store the recorded topics
        recorded_data = {topic: [] for topic in topic_names}

        is_first_time_stamp = {}
        first_time_stamp = {}

        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()



            if topic not in first_time_stamp:
                first_time_stamp[topic] = timestamp
                is_first_time_stamp[topic] = False

            relative_time_stamp = (timestamp - first_time_stamp[topic]) / 1e9

            if topic in topic_names:
                # check which topic is read
                if self.type_map[topic] == "omni_msgs/msg/HIDState":
                    # Deserialize HIDState (global_state)
                    global_state = deserialize_message(data, HIDState)
                    recorded_data[topic].append([
                        relative_time_stamp,                                                                                                    # time (0)
                        global_state.pose.position.x, global_state.pose.position.y, global_state.pose.position.z,                               # position (1-4)
                        global_state.velocity.x, global_state.velocity.y, global_state.velocity.z,                                              # velocity (4-7)
                        global_state.force.x, global_state.force.y, global_state.force.z                                                        # force (7-10)
                    ])

                
                elif self.type_map[topic] == "omni_msgs/msg/FrenetState":
                    # Deserialize FrenetState (frenet_state)
                    frenet_state = deserialize_message(data, FrenetState)
                    flattened_frenet_state = np.concatenate([
                        np.array([relative_time_stamp]),                        # time (0)
                        frenet_state.orthogonal_projection_position,            # orthogonal projection position (3x1) (1-4)
                        frenet_state.frenet_state,                              # frenet state (6x1) (4-10)
                        frenet_state.frenet_matrix                              # frenet matrix (9x1) (10-19)
                    ])

                    recorded_data[topic].append(flattened_frenet_state)

                
                elif self.type_map[topic] == "geometry_msgs/msg/Vector3":
                    control_force = deserialize_message(data, Vector3)
                    recorded_data[topic].append([
                        relative_time_stamp, 
                        control_force.x, control_force.y, control_force.z
                    ])

                elif self.type_map[topic] == "geometry_msgs/msg/WrenchStamped":
                    human_force = deserialize_message(data, WrenchStamped)
                    recorded_data[topic].append([
                        relative_time_stamp, 
                        human_force.wrench.force.x, human_force.wrench.force.y, human_force.wrench.force.z
                    ])


                elif self.type_map[topic] == "omni_msgs/msg/HIDFeedback":
                    feedback = deserialize_message(data, HIDFeedback)
                    recorded_data[topic].append([
                        relative_time_stamp,                                                                # time
                        feedback.desired_force.x, feedback.desired_force.y, feedback.desired_force.z,       # calculated feedack force 
                    ])
        
        # Return the recorded data as numpy arrays for each topic
        return {topic: np.array(recorded_data[topic]) for topic in recorded_data}
    

    def read_frenet_state(self, topic_name):
        recorded_frenet_state = []
        is_first_time_stamp = True
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            if is_first_time_stamp:
                first_time_stamp = timestamp
                is_first_time_stamp = False
            if topic == topic_name:
                frenet_state = deserialize_message(data, FrenetState)
                recorded_frenet_state.append([(timestamp - first_time_stamp) / 1e9,                         # time   
                                              frenet_state.orthogonal_projection_position,                  # orthogonal projection position    (3x1)
                                              frenet_state.frenet_state,                                    # frenet state                     (6x1)  
                                              frenet_state.frenet_matrix])                                  # frenet matrix                     (9x1) 
        return np.array(recorded_frenet_state)
    

    # def save_to_matlab(data, filename="rosbag_data.mat"):
    #     """
    #     Speichert die ROS-Bag-Daten als .mat-Datei für MATLAB.
        
    #     :param data: Dictionary mit den ROS-Topics als Schlüssel und NumPy-Arrays als Werte.
    #     :param filename: Name der zu speichernden .mat-Datei.
    #     """
    #     # MATLAB mag keine Dictionaries mit Strings als Keys -> Umwandeln
    #     matlab_data = {key.replace("/", "_"): value for key, value in data.items()}

    #     file_path = "/home/irs/jonas_ws/src/Rosbags/record_motion/"
    #     # Speichern als .mat Datei
    #     scipy.io.savemat(file_path + filename, matlab_data)

    #     print(f"Gespeichert als {filename}")


    def sync_data(self, t1, t2, f2, kind='linear'):
        """
        Interpoliert f2 auf die Zeitpunkte von t1 und berechnet f1 - f2.
        
        Inputs:
            t1 : Zeitstempel 1 (z.B. automation)
            f1 : Werte 1
            t2 : Zeitstempel 2 (z.B. human)
            f2 : Werte 2
            kind : Interpolationsart ('linear', 'cubic', etc.)
            
        Returns:
            t_common : Zeitpunkte (t1)
            delta_f : Differenz (f1 - interpoliertes f2)
        """
        # Interpolation f2 -> t1
        interp_f2 = interp1d(t2, f2, kind=kind, fill_value="extrapolate")
        f2_interp = interp_f2(t1)

        return f2_interp

    def plot_figures(self, data):

        #### position ####

        plt.figure
        plt.subplot(projection='3d')
        plt.plot(data["/master/state"][:,1], data["/master/state"][:,2], data["/master/state"][:,3])

        plt.figure()
        plt.plot(data["/master/state"][:,1], data["/master/state"][:,3])
        plt.xlabel("x")
        plt.ylabel("z")
        plt.title("Global State x-z plane")
        plt.legend()

        plt.figure()
        plt.plot(data["/master/state"][:,1], data["/master/state"][:,2], label="x-y")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Global State x-y plane")
        plt.legend()

        #### frenet state ####

        plt.figure()        # distance
        plt.subplot(3,1,1)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,4], label="d_t")
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,5], label="d_n")
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,6], label="d_b")
        plt.legend()

        plt.figure()        # velocity
        plt.subplot(3,1,1)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,7], label="v_t")
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,8], label="v_n")
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,9], label="v_b")
        plt.legend()

        #### force feedback ####

        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,1], label="F_x_automation")
        # if data["/master/feedback_human"].size != 0:
        #     plt.plot(data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,1], label="F_x_human")
        #     f_human_inter_x = self.sync_data(data["/master/feedback"][:,0], data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,1])
        #     plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,1] + f_human_inter_x, '--',label="F_x - combined")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,2)
        plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,2], label="F_y_automation")
        # if data["/master/feedback_human"].size != 0:
        #     plt.plot(data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,2], label="F_y_human")
        #     f_human_inter_y = self.sync_data(data["/master/feedback"][:,0], data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,2])
        #     plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,2] + f_human_inter_y, '--',label="F_y - combined")
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3,1,3)
        plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,3], label="F_z_automation")
        # if data["/master/feedback_human"].size != 0:
        #     plt.plot(data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,3], label="F_z_human")
        #     f_human_inter_z = self.sync_data(data["/master/feedback"][:,0], data["/master/feedback_human"][:,0], data["/master/feedback_human"][:,3])
        #     plt.plot(data["/master/feedback"][:,0], data["/master/feedback"][:,3] + f_human_inter_z, '--', label="F_z - combined")
        plt.legend()
        plt.grid(True)


        ### force human ###
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(data["/human_force_base_frame"][:,0], data["/human_force_base_frame"][:,1], label="F_x_human")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,2)
        plt.plot(data["/human_force_base_frame"][:,0], data["/human_force_base_frame"][:,2], label="F_y_human")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,3)
        plt.plot(data["/human_force_base_frame"][:,0], data["/human_force_base_frame"][:,3], label="F_z_human")
        plt.legend()
        plt.grid(True)


        ### controller force ###
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(data["/frenet/force_feedback"][:,0], data["/frenet/force_feedback"][:,1], label="MPC_force_x")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,2)
        plt.plot(data["/frenet/force_feedback"][:,0], data["/frenet/force_feedback"][:,2], label="MPC_force_y")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,3)
        plt.plot(data["/frenet/force_feedback"][:,0], data["/frenet/force_feedback"][:,3], label="MPC_force_z")
        plt.legend()
        plt.grid(True)



        #### frenet frames ####
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,10], label="T_x")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,11], label="T_y")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,12], label="T_z")
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,13], label="N_x")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,14], label="N_y")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,15], label="N_z")

        plt.subplot(3,1,3)
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,16], label="B_x")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,17], label="B_y")
        plt.plot(data["/master/frenet_state"][:,0], data["/master/frenet_state"][:,18], label="B_z")

        plt.show()
    

def main():
    bag_folder = "/home/irs/frenet_ws/src/Rosbags"
    bag_file = "20251201_165513-balint_medical_3d_curve.bag" 

    reader = RosbagReader(os.path.join(bag_folder, bag_file))

    data = reader.read_ros_bag(["/master/state", "/master/feedback", "/master/frenet_state", "/human_force_base_frame", "/frenet/force_feedback"])  # "/master/feedback_human", "/master/frenet_state_human"

    for key, value in data.items():
        print(f"Topic: {key}, Shape: {value.shape}, Type: {type(value)}")

    reader.plot_figures(data)

    data_fixed = {
    "master_state": data["/master/state"],
    "master_feedback": data["/master/feedback"],
    "master_frenet_state": data["/master/frenet_state"],
    #"master_feedback_human": data["/master/feedback_human"],
    #"master_frenet_state_human": data["/master/frenet_state_human"]
    }

    # import scipy.io
    # scipy.io.savemat("/home/irs/jonas_ws/simulation_data/rosbag_data_ausarbeitung_mmi_final.mat", data_fixed)

if __name__ == "__main__":
    main()