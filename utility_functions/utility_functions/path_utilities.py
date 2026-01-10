import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

from .trajectorie_definition import TrajectoryDefinitions

class ReferencePath:
    def __init__(self, num_points=5000, trajectory_type = 1, num_interp_points = 100, segment_factor = 0.0001, kappa_treshhold = 1e-4, comp_type = 0, trajectory_scaling = 1.0, frame_type = 1):
        """
        Initialize the ReferencePath object
        """
        self.num_points = num_points
        self.trajectories_obj = TrajectoryDefinitions(num_points)

        self.arc_length = None
        self.curve_param = None
        self.ref_path = None
        self.inter_ref_path = None

        self.trajectory_type = trajectory_type
        self.trajectory_scaling = trajectory_scaling

        self.define_reference_path()

        self.init_frame_computation_properties(num_interp_points, segment_factor, kappa_treshhold)
        
        if frame_type == 0:
            self.traj_matrix = self.compute_trajectory_matrix_frenet(computation_type=comp_type)
        else:
            self.traj_matrix = self.compute_trajectory_matrix_bishop(computation_type=comp_type)

         # build trajectory matrix
        # objetct -- value
        # 0       -- arc length
        # 1-4     -- cartesian position
        # 4       -- curvature
        # 5       -- torsion
        # 6- 15   -- Frame vectors [F1; F2; F3]
       

    def init_frame_computation_properties(self, num_interp_points, segment_factor, kappa_treshhold):
        self.F_prev = np.eye(3)  #np.array([[0,1,0], [1,0,0], [0,0,-1]]) #                                # initial Frenet frame -- arbitrary
        self.numInterpPoints = num_interp_points                # number of points for the segment interpolation    
        self.kappa_treshhold = kappa_treshhold                  # lower treshhold for curvature (to avoid numerical errors)
        self.segment_factor = segment_factor                    # factor for length of the segment around the closest point (segment_factor * arc_length)
    

    
    def define_reference_path(self):
        """
        Define the reference path based on the path type.

        Inputs:
            - self.trajectory_type: string representing the path type
        """
        
        if self.trajectory_type == 0:
            ref_path = self.trajectories_obj.medical_incision_curve()
        elif self.trajectory_type == 1:
            ref_path = self.trajectories_obj.medical_incision_curve_3d()
        elif self.trajectory_type == 2:
            ref_path = self.trajectories_obj.rehab()
        elif self.trajectory_type == 3:
            ref_path = self.trajectories_obj.rehab_3D()
        elif self.trajectory_type == 4:
            ref_path = self.trajectories_obj.rehab_3D_curve()
        elif self.trajectory_type == 5:
            ref_path = self.trajectories_obj.lines()
        elif self.trajectory_type == 6:
            ref_path = self.trajectories_obj.helix()
        elif self.trajectory_type == 7:
            ref_path = self.trajectories_obj.sin_curve()
        elif self.trajectory_type == 8:
            ref_path = self.trajectories_obj.practical()
        elif self.trajectory_type == 9:
            ref_path = self.trajectories_obj.standard_line()
        else:
            raise ValueError('Invalid path type')
   
        self.ref_path = ref_path * self.trajectory_scaling


    
    def get_trajectory(self):
        """
        Return the reference trajectory, interpolated reference trajectory and arc length
        """
        return self.ref_path, self.inter_ref_path, self.arc_length
    

    def get_idx_from_arc_length(self, s):
        """
        Get the current idx of the trajectory matrix from the arc length. Use a binary search to find the nearest point on the reference path.
        """

        idx = self.find_nearest_idx(self.traj_matrix[0,:], s)

        return idx


    def compute_trajectory_matrix_frenet(self, computation_type=0):
            """
            Compute the trajectory matrix for the reference path.
            s, x, y, z, kappa, tau, T, N, B -> 15 x n matrix 

            Inputs:
                - computation_type: 0 for discrete calculations, 1 for polynomial calculations
            Output:
                - traj_matrix: trajectory matrix (s; pos; kappa; tau; T; N; B) [15 x n numpy array]
            """
            if computation_type == 0:
                s_vals, points, kappa_vals, tau_vals, F_vals = self.compute_path_properties_discrete()
            else:
                s_vals, points, kappa_vals, tau_vals, F_vals = self.compute_path_properties_poly()

            n = self.num_points

            traj_matrix = np.zeros((15, n))

            traj_matrix[0,:] = s_vals
            traj_matrix[1:4,:] = points
            traj_matrix[4,:] = kappa_vals
            traj_matrix[5,:] = tau_vals
            traj_matrix[6:15, :] = F_vals

            self.arc_length = s_vals[-1]

            return traj_matrix
    

    def compute_trajectory_matrix_bishop(self, computation_type=0):
        """
        Calculate the trajectory matrix using the Bishop frame.
        """
        n = self.num_points
        s_vals, points, kappa_vals, tau_vals, F_vals = self.compute_path_properties_discrete()

        T = F_vals[0:3,:]

        N_0 = np.zeros([3, len(T[0,:])])     # init bishop frame with first frenet frame
        N_0[:,0] = F_vals[3:6,0]             # first normal vector

        N_1 = np.zeros([3, len(T[0,:])])     
        N_1[:,0] = F_vals[6:9,0]             # first binormal vector

        for i in range(0, len(T[0,:])-1):
            RA = np.cross(T[:,i], T[:,i+1])         # temporal Rotation axis       

            if np.linalg.norm(RA) < 1e-4:
                N_0[:,i+1] = N_0[:,i]
                N_1[:,i+1] = N_1[:,i]
            else:
                RA = RA / np.linalg.norm(RA)
                theta = np.arccos(np.dot(T[:,i], T[:,i+1]))

                rot = R.from_rotvec(RA * theta)       # rotation matrix
                rot_matrix = rot.as_matrix()

                N_0[:,i+1] = rot_matrix @ N_0[:,i]
                N_1[:,i+1] = np.cross(T[:,i+1], N_0[:,i+1])


        T_prime = np.gradient(T, axis=1)            # gradient of T
        k1 = np.einsum('ij,ij->j', T_prime, N_0)         # dot product of T and N_0
        k2 = np.einsum('ij,ij->j', T_prime, N_1)         # dot product of T and N_1


        ds = np.gradient(s_vals)         # gradient of s_vals
        k1 = k1 / ds #*100
        k2 = k2 / ds #*100

        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.plot(kappa_vals)
        # plt.subplot(2, 1, 2)
        # #fac = max(kappa_vals) / max(k1)
        # plt.plot(np.sqrt(k1**2 + k2**2))
        # plt.show()


        traj_matrix = np.zeros((15, n))
        traj_matrix[0,:] = s_vals
        traj_matrix[1:4,:] = points
        traj_matrix[4,:] = k1
        traj_matrix[5,:] = k2
        traj_matrix[6:9, :] = T
        traj_matrix[9:12, :] = N_0
        traj_matrix[12:15, :] = N_1

        self.arc_length = s_vals[-1]

        self.traj_matrix = traj_matrix

        return traj_matrix


    def interpolate_points(self, refPath):
        """
        Interpolate the points on the reference path to obtain a smooth equally distributed path. 

        Inputs:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]

        Output:
            - traj: tuple containing the knots, coefficients, and degree of the spline
        """
        x = refPath[0,:]
        y = refPath[1,:]
        z = refPath[2,:]
        
        tck, u = splprep([x, y, z], s=0)        # spline interpolation 

        u_fine = np.linspace(0, 1, self.num_points)      # use more points to calc arc length
        x_spline, y_spline, z_spline = splev(u_fine, tck)

        dx = np.diff(x_spline)
        dy = np.diff(y_spline)
        dz = np.diff(z_spline)
        ds = np.sqrt(dx**2 + dy**2 + dz**2)  
        arc_lengths = np.cumsum(ds)         # Cumulative arc lengths
        arc_length = arc_lengths[-1]   

        traj = np.vstack([x_spline, y_spline, z_spline])

        self.curve_param = tck
        #self.arc_length = arc_length

        self.inter_ref_path = traj

    
    def find_nearest_idx(self, sorted_array, value):
        '''
        1D nearest index search using binary search

        Input:
            - sorted_array: numpy array containing the sorted values (path_progress) [nx1]
            - value: value to search for
        Output:
            - idx: index of the nearest value
        '''
        idx = np.searchsorted(sorted_array, value)

        if idx == 0: 
            return idx
        if idx == len(sorted_array):
            return idx - 1
        
        before = sorted_array[idx - 1]
        after = sorted_array[idx]

        if abs(value - before) <= abs(after - value):
            return idx - 1
        else:
            return idx


    # ========== polynomial calculations ==========

    def compute_path_properties_poly(self):
        """
        Compute the arc_len, curvature, torsion and frame of the reference path using polynomial calculations/approximations. 
        """
        F_vals = np.zeros((9, self.num_points))  
        kappa_vals = np.zeros(self.num_points)
        tau_vals = np.zeros(self.num_points)
        points = np.zeros((3, self.num_points))

        s_vals = np.zeros(self.num_points)

        for i in range(self.num_points):
            seg_onRef = self.compute_segment_for_ref_point(i) 
            F, kappa, tau, realPoint = self.calculate_frenet_properties(seg_onRef)

            F_vals[0:3,i] = F[:,0]
            F_vals[3:6,i] = F[:,1]
            F_vals[6:9,i] = F[:,2]
            kappa_vals[i] = kappa
            tau_vals[i] = tau
            points[:,i] = realPoint
        
        s_vals = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=1)**2, axis=0)))
        s_vals = np.insert(s_vals, 0, 0)

        return s_vals, points, kappa_vals, tau_vals, F_vals


    def calculate_frenet_properties(self, selected_points):
        '''
        Calculate the Frenet frame (properties) for a given point on the reference path 
        -- Method used: Spline fitting using 5 points on the reference path

        Input:
            - selected_points: (interpolated) segment on the reference path [3xN numpy matrix]
            - closestPointIdx: index of the point which is closest to the reference path [int]
        Output:
            - F: Frenet frame matrix (contains the tangent, normal, and binormal vectors) [6x6 numpy matrix]
            - kappa: curvature
            - tau: torsion
        '''

        tck, _ = splprep(selected_points, s=0, k=3)  # k=3 for segond derivative

        # evaluate spline at the closest point of the segment using the indices
        u = self.u_local                       #closestPointIdx/self.numInterpPoints
        deriv_1 = np.array(splev(u, tck, der=1))    # derivations
        deriv_2 = np.array(splev(u, tck, der=2)) 
        deriv_3 = np.array(splev(u, tck, der=3))  

        # calculate curvature: ||r'(t) × r''(t)|| / ||r'(t)||^3
        cross_product = np.cross(deriv_1, deriv_2)
        kappa = np.linalg.norm(cross_product) / np.linalg.norm(deriv_1) ** 3


        real_point = np.array(splev(u, tck))


        if kappa < self.kappa_treshhold:           # extended frenet frame: for deviation == small -> use previous frame
            
            ##### use previous frame #####
            #F = self.F_prev

            T = deriv_1 / np.linalg.norm(deriv_1) 

            ##### use gram schmidt #####
            N, B = self.gram_schmidt(T, self.F_prev[:,1], self.F_prev[:,2])

            ##### use arbitrary system #####
            # N, B = self.arbitrary_system(T)

            tau = 0.0

            F = np.column_stack([T, N, B]) 


        else:
            T = deriv_1 / np.linalg.norm(deriv_1)       # normal tangent vector

            N = deriv_2 / np.linalg.norm(deriv_2)       # normal vector

            B = np.cross(T, N)                          # binormal vector

            F = np.column_stack([T, N, B])              # frenet frame -- rotation matrix

            self.F_prev = F

            # calculate torsion: det(r'(t), r''(t), r'''(t)) / ||r'(t) × r''(t)||^2
            numerator = np.dot(np.cross(deriv_1, deriv_2), deriv_3)
            denominator = np.linalg.norm(cross_product) ** 2
            tau = numerator / denominator if denominator > 0 else 0.0

        return F, kappa, tau, real_point
    

    def compute_segment_for_ref_point(self, idx):
        '''
        Interpolate segment around given point on the reference path

        Input:
            - idx: index of the point on the reference path

        Output:
            - closestPoint: closest point to reference Path [3x1 numpy array]
            - idx: index of the nearest point on the reference path [int]
        '''

        seg_onRef = self.interpolate_path_segment(idx)

        

        return seg_onRef
    

    def interpolate_path_segment(self, ref_path_p_idx):
        '''
        Interpolate the reference path using a B-Spline, the resulting point has d_t = 0

        Input:
            - numPoints: number of points on the interpolated path
        Output:
            - ref_path_interpolated: 3xnumPoints numpy array
        '''
        num = 10 # number of points to the left and right of the reference point 
        u_idx = ref_path_p_idx
        
        if ref_path_p_idx < num:
            ref_path_p_idx = num
        elif ref_path_p_idx > self.ref_path.shape[1] - num:
            ref_path_p_idx = self.ref_path.shape[1] - num
            
        selected_points = self.ref_path[:, ref_path_p_idx-num:ref_path_p_idx+num+1]  

        tck, _ = splprep(selected_points, s=5, k=3)              # fitting                       
        
        u_new = np.linspace(0.0, 1.0, self.numInterpPoints)     # generate new points using the spline
        ref_path_interpolated = np.array(splev(u_new, tck))

        interpolatedPointIdx = self.find_closest_point_on_segment(self.ref_path[:, ref_path_p_idx], ref_path_interpolated)

        if u_idx < num:
            self.u_local = u_idx/num / 2
        elif u_idx > self.ref_path.shape[1] - num:
            self.u_local = (u_idx - (self.ref_path.shape[1] - num))/num *0.5 + 0.5
        else:
            self.u_local = interpolatedPointIdx/self.numInterpPoints

        return ref_path_interpolated
    
    
    def find_closest_point_on_segment(self, global_point, segment):
        '''
        Find the closest point on the given segment to the global point

        Input:
            - global_point: [3x1 numpy array]
            - segment: set of points [3xN numpy array]

        Output:
            - closestPoint: [3x1 numpy array]
            - indices: index of the nearest point on the segment [int]
        '''

        tree = cKDTree(segment.T)
        _, indices = tree.query(global_point, k=1)       # k = 1: return only the nearest point

        return indices
    
    
    # ========== discrete calculations ==========

    def compute_path_properties_discrete(self):
        """
        Compute the arc_len, curvature, torsion and frame of the reference path using discrete calculations/approximations. 
        """

        s_vals = self.compute_arc_length(self.ref_path)
        dr, ddr = self.compute_derivatives(self.ref_path, s_vals)

        F_vals = self.compute_frenet_frame_discrete(dr, ddr)

        kappa_vals = self.discrete_curvature(self.ref_path)
        tau_vals = self.discrete_torsion(s_vals, F_vals[3:6,:], F_vals[6:9,:])
        
        points = self.ref_path

        return s_vals, points, kappa_vals, tau_vals, F_vals


    def compute_frenet_frame_discrete(self, dr, ddr):
        '''
        Compute the Frenet frame for the given derivatives of a trajectory.
        If the curvature is zero (smaller than certain treshhold), the normal and binormal vectors of the previous frame are adapted using the Gram-Schmidt process.

        Input:
            - dr: first derivative [3 x n numpy array]
            - ddr: second derivative [3 x n numpy array]
        Output:
            - F_array: Frenet frame array (T; N; B) [9 x n numpy array]
        '''
        F_save = np.zeros((9, dr.shape[1]))
        F_prev = np.zeros([9,])
        F_prev[0:3] = self.F_prev[:,0]
        F_prev[3:6] = self.F_prev[:,1]
        F_prev[6:9] = self.F_prev[:,2]

        for i in range(dr.shape[1]):

            T = dr[:, i] / np.linalg.norm(dr[:, i])

            if np.linalg.norm(ddr[:, i]) < 1e-3:
                N, B = self.gram_schmidt(T, F_prev[3:6], F_prev[6:9])

                F = np.vstack((T, N, B)).reshape(-1)
            else:
                N = ddr[:, i] / np.linalg.norm(ddr[:, i])
                B = np.cross(T, N)

                F = np.vstack((T, N, B)).reshape(-1)
                F_prev = F
                
            F_save[:,i] = F
            
        return F_save


    def compute_arc_length(self, points):
        """
        Calculate the cumulative arc length of a 3D trajectory using the Euclidean distance between consecutive points.
        
        Input:
            - points: 3D trajectory points [3 x n numpy array]
        Output:
            - cumulative_length: cumulative arc length [n x 1 numpy array]
        """
        segment_lengths = np.linalg.norm(np.diff(points, axis=1), axis=0)   # distance between consecutive points

        cumulative_length = np.insert(np.cumsum(segment_lengths), 0, 0)     # fill the first element with 0

        return cumulative_length


    def compute_derivatives(self, points, arc_len):
        """
        Compute the first and second derivative (dr/ds and dr^2/ds^2) of the 3D trajectory using central differences.
        Can cope with non-uniformly distributed points (varying arc length).
        
        Input:
            - points: 3D trajectory points [3 x n numpy array]
            - arc_len: cumulative arc length [n x 1 numpy array]
        Output:
            - dr: first derivative [3 x n numpy array]
            - ddr: second derivative [3 x n numpy array]
        """
        n = points.shape[1]
        
        dr = np.zeros((3, n))
        ddr = np.zeros((3, n))
        
        for i in range(1, n - 1):
            ds_back = arc_len[i] - arc_len[i - 1]
            ds_fwd = arc_len[i + 1] - arc_len[i]
            ds_total = ds_back + ds_fwd

            # first derivative)
            dr[:, i] = (ds_back**2 * (points[:, i + 1] - points[:, i]) +
                                    ds_fwd**2 * (points[:, i] - points[:, i - 1])) / (ds_fwd * ds_back * ds_total)

            # second derivative
            ddr[:, i] = 2 * (
                (points[:, i + 1] - points[:, i]) / ds_fwd -
                (points[:, i] - points[:, i - 1]) / ds_back
            ) / ds_total

        # edge cases
        # forward difference for first point
        ds_fwd = arc_len[1] - arc_len[0]
        dr[:, 0] = (points[:, 1] - points[:, 0]) / ds_fwd

        ds1 = arc_len[1] - arc_len[0]
        ds2 = arc_len[2] - arc_len[1]
        ddr[:, 0] = 2 * (
            (points[:, 2] - points[:, 1]) / ds2 -
            (points[:, 1] - points[:, 0]) / ds1
        ) / (ds1 + ds2)

        # backward difference for last point
        ds_back = np.linalg.norm(points[:, -1] - points[:, -2])
        dr[:, -1] = (points[:, -1] - points[:, -2]) / ds_back

        ds1 = arc_len[-1] - arc_len[-2]
        ds2 = arc_len[-2] - arc_len[-3]
        ddr[:, -1] = 2 * (
            (points[:, -1] - points[:, -2]) / ds1 -
            (points[:, -2] - points[:, -3]) / ds2
        ) / (ds1 + ds2)

        return dr, ddr


    def discrete_curvature(self, points):
        """
        Calculate the curvature of a 3D trajectory using a discrete approach.
        Approximation using the triangle area between three consecutive points: Curvature = (2 * area / (product of norms))
        
        Input:
            - points: 3D trajectory points [3 x n numpy array]
        Outptu:
            - curvature: curvature values [n x 1(,) numpy array]
        """
        n = len(points[0])
        curvature = np.zeros(n)
        
        for i in range(1, n - 1):
            p1 = points[:, i - 1]
            p2 = points[:, i]
            p3 = points[:, i + 1]
            
            v1 = p2 - p1    # vectors between the points
            v2 = p3 - p2
            
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            cross_prod = np.cross(v1, v2)           # triangle area using cross product
            area = np.linalg.norm(cross_prod) 
            
            if norm_v1 > 0 and norm_v2 > 0:         
                curvature[i] = 2 * area / (norm_v1 * norm_v2 * np.linalg.norm(p3 - p1))

        # edges of curvature
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]

        return curvature
    

    def discrete_torsion(self, s, N, B):
        '''
        Calculate torsion using a discrete approach.
        Approximation using the scalar product of the derivative of the binormal vector and the normal vector: Tau = - (dB/ds * N)

        Input:
            - B: binormal vector [3 x n numpy array]
            - N: normal vector [3 x n numpy array]
            - s: arc length [n x 1 numpy array]
        Output:
            - tau: torsion values [n x 1 numpy array]
        '''
        n = B.shape[1]
        dB_ds = np.zeros((3, n))

        for i in range(1, n - 1):
            h1 = s[i] - s[i - 1]
            h2 = s[i + 1] - s[i]

            dB_ds[:, i] = (
                            h1**2 * (B[:, i + 1] - B[:, i]) + h2**2 * (B[:, i] - B[:, i - 1])
                            ) / (h1 * h2 * (h1 + h2))



        # edges of dB
        dB_ds[:, 0] = (B[:, 1] - B[:, 0]) / (s[1] - s[0])
        dB_ds[:, -1] = (B[:, -1] - B[:, -2]) / (s[-1] - s[-2])

        # Dot product with N 
        tau = - np.einsum('ij,ij->j', dB_ds, N)

        # edges of tau 
        tau[0:2] = tau[3]
        tau[-2:] = tau[-3]

        jump_idx = self.detect_tau_jumps(tau)

        tau = self.fill_tau_with_next_valid(tau, jump_idx)

        return tau
    

    def detect_tau_jumps(self, tau, threshold_factor=100):
        """
        Erkennt große Sprünge in der Torsion (tau).

        Input:
            tau: Array der Torsion [n]
            threshold_factor: Multiplikator über dem Mittelwert der Differenzen
        Output:
            jump_indices: Indizes, an denen ein Sprung erkannt wurde
        """
        # 1. Erste Differenz (Unterschied zwischen aufeinanderfolgenden Werten)
        dtau = np.abs(np.diff(tau))  # Länge: n-1

        # 2. Schwellenwert definieren (z. B. 10x Median oder Mittelwert)
        threshold = threshold_factor * np.mean(dtau)

        # 3. Indizes mit auffälligen Sprüngen
        jump_indices = np.where(dtau > threshold)[0]

        return jump_indices
    

    def fill_tau_with_next_valid(self, tau, jump_indices):
        """
        Ersetzt die Werte an jump_indices durch den nächsten gültigen (nicht gesprungenen) tau-Wert.

        Parameter:
            tau: np.array mit Torsionswerten
            jump_indices: Liste oder Array mit Indizes, die ersetzt werden sollen
        Rückgabe:
            tau_filled: np.array mit ersetzten Werten
        """
        tau_filled = tau.copy()
        n = len(tau)

        for i in jump_indices:
            # Suche nach dem nächsten gültigen Wert
            next_valid = None
            for j in range(i + 1, n):
                if j not in jump_indices:
                    next_valid = tau[j]
                    break

            # Falls gültiger Wert gefunden, ersetze
            if next_valid is not None:
                tau_filled[i] = next_valid
            else:
                # kein späterer Wert verfügbar → lasse unverändert oder mit NaN
                tau_filled[i] = np.nan  # oder: tau[i]

        return tau_filled


    
    # ========= extended Frenet Frame ==========

    def gram_schmidt(self, t, n_old, b_old):
        """
        Adapt normal and binormal vector using the Gram-Schmidt process, if the curvature of the trajectorie is zero.
        Removes components of the system which are parallel to the tangent vector.
        """
        n = n_old - np.dot(n_old, t) * t        # adapt normal vector
        n = n /  np.linalg.norm(n)
        
        b = np.cross(t, n)  
        b = b /  np.linalg.norm(b)
        
        return n, b