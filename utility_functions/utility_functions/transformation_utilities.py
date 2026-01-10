import numpy as np 
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

from scipy.spatial import cKDTree


class GeometricTransformation:
    #def __init__(self, referenceTrajectory, splineParameters, arcLength):
    def __init__(self, path_obj, system_period, d_t_tresh):

        self.path_obj = path_obj                
        self.traj_matrix = path_obj.traj_matrix             # path matrix
        self.ref_path = self.traj_matrix[1:4, :]            # reference path

        self.tree = cKDTree(self.ref_path.T)    # KDTree for fast nearest neighbor search

        self.d_t_treshhold = d_t_tresh          # numeric treshhold for d_t
        self.prev_closest_point = None

        self.s_type = 1             # 0 = dynamic progress, 1 = geometric progress
        self.s_prog = 0             # frame progress
        self.s_dot = 0              # frame velocity

        self.s_prog_geo = 0
        self.s_prog_dyn = 0

        self.dt = system_period


    def global_to_frenet(self, state_vector_global):
        '''
        Convert a global point to the Frenet frame

        Input:
            - state_vector_global: [6, numpy array] (x, y, z, v_x, v_y, v_z)
        Output:
            - stateVectorFrenet: [6x1 numpy array] (d_t, d_n, d_b, v_t, v_n, v_b)
            - F: Frenet frame matrix (contains the tangent, normal, and binormal vectors) [6x6 numpy array]
            - kappa: curvature [float]
            - tau: torsion [float]
            - orth_proj: orthogonal projection of the global point onto the reference path [3x1 numpy array]
            - s: progress on the reference path [float]
        '''

        self.global_point = state_vector_global[0:3]

        indice, orth_proj = self.find_closest_point_on_refPath(state_vector_global[0:3]) 

        #orth_proj = self.ref_path[:, indice]

        s = self.traj_matrix[0, indice]                             # get trajectory properties from path obj
        pos = self.traj_matrix[1:4, indice]
        kappa = self.traj_matrix[4, indice]
        tau = self.traj_matrix[5, indice]
        T = self.traj_matrix[6:9, indice]
        N = self.traj_matrix[9:12, indice]
        B = self.traj_matrix[12:15, indice]
        
        F = np.column_stack([T, N, B])

        d_FF = F.T @ (state_vector_global[0:3] - orth_proj)         # tracking error

        if np.abs(d_FF[0]) < self.d_t_treshhold:                    # numeric treshhold for d_t
            d_FF[0] = 0

        v_FF = F.T @ state_vector_global[3:6]                       # velocity in Frenet frame    


        self.geometric_curve_progress(orth_proj)

        self.dynamic_curve_progress(F, state_vector_global, v_FF[0], orth_proj, kappa)

        if self.s_type == 0:
            self.s_prog = self.s_prog_dyn
        else:
            self.s_prog = self.s_prog_geo + self.s_dot * self.dt

        return np.concatenate((d_FF, v_FF)).reshape(6,), F, orth_proj, self.s_dot, s
    

    def dynamic_curve_progress(self, FrenetMatrix, globalStateVector, v_tang, pointOnRef, kappa):
        '''
        Calculate the frame progress on the reference path based on the tracking error, the velocity, and the curvature.
        (Changing velocity for different radia of the reference path)
        '''

        dot_prod = np.dot(FrenetMatrix[:,1], globalStateVector[:3] - pointOnRef)

        correction_factor = 1 - kappa * dot_prod

        self.s_dot = v_tang / correction_factor
        
        self.s_prog_dyn += self.s_dot * self.dt

    
    def geometric_curve_progress(self, new_closest_point):
        '''
        Calculate the progress on the reference path using eucledean distance calculations.

        Input:
            - new_closest_point: closest point on the reference path
        Output:
            - s_prog_geo: progress on the reference path
        '''
        if self.prev_closest_point is None:
            self.prev_closest_point = new_closest_point

        s_step = np.linalg.norm(new_closest_point - self.prev_closest_point)
    
        self.s_prog_geo += s_step

        self.prev_closest_point = new_closest_point

    
    def get_path_progress(self):
        return self.s_prog, self.s_dot
    

    def find_closest_point_on_refPath(self, global_point):
        '''
        Find the closest point on the reference path for the given global point.
        Caclulate the orthogonal projection of the global point onto the reference path. 

        Input:
            - global_point: [3x1 numpy array]

        Output:
            - orth_proj: orthogonal projection of the global point onto the reference path [3x1 numpy array]
            - indices: index of the nearest point on the reference path [int]
        '''

        _, indices = self.tree.query(global_point, k=2)

        closest_points = self.ref_path[:, indices]

        orth_proj = self.orthogonal_projection_on_line(closest_points[:,0], closest_points[:,1], global_point)

        return indices[0], orth_proj
    

    def orthogonal_projection_on_line(self, p1, p2, global_point):
        """
        Caclulate the orthogonal projection of a 'point' on the line defined by 'p1' and 'p2'
        
        Input:
            - p1: [3x1 numpy array]
            - p2: [3x1 numpy array]
            - global_point: [3x1 numpy array]

        Output:
            - projection: [3x1 numpy array] orthogonal projection of the global point onto the line defined by p1 and p2
        """
        line_vector = p2 - p1
        point_vector = global_point - p1
        t = np.dot(point_vector, line_vector) / np.dot(line_vector, line_vector)  # Skalarer Anteil
        projection = p1 + t * line_vector

        return projection
    

    
        
    