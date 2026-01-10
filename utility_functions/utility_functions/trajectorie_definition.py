import numpy as np

class TrajectoryDefinitions:
    """
    A class to define and manage trajectory-related operations.
    """

    def __init__(self, num_points = 5000):
        """
        Initialize the TrajectoryDefinitions class.
        """
        self.num_points = num_points

    def medical_incision_straight(self):
        """
        Straight line which represents a medical incision with straight movement after the incision.

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        length = 0.3

        x = np.linspace(0, length, self.num_points)
        y = np.linspace(0, length, self.num_points)
        z = np.linspace(2, 2-length, self.num_points)

        traj = np.vstack([x, y, z])

        return traj


    def medical_incision_curve(self):
        """
        Curve which represents a medical incision including curve movement after the incision.

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        
        # Parameter
        radius_1 = 0.1    
        radius_2 = 0.05        # radios of the semi circle       
        segment_length = 0.2     # length of straight line

        # Semi circle
        x1 = np.linspace(0, segment_length, self.num_points // 5 * 2)
        t1 = np.linspace(np.pi+0.05, 3/2*np.pi, self.num_points // 5 * 2)    # angle of circle
        z1 = radius_1 + radius_1 * np.sin(t1)
        y1 = np.zeros_like(x1)

        # Straight 
        x2 = np.linspace(segment_length + 0.0001, segment_length*1.5, self.num_points // 5)
        y2 = np.zeros_like(x2)
        z2 = np.zeros_like(x2)
              
        # semi circle
        t3 = np.linspace(np.pi/2*3, np.pi*5/2, self.num_points // 5 * 2)    # angle of circle
        x3 = np.cos(t3) * radius_2 * 1.5 + segment_length*1.5 + 0.00001
        y3 = np.zeros_like(x1)
        z3 = np.sin(t3) * radius_2 + radius_2 

        x = np.concatenate([x1, x2, x3])
        y = np.concatenate([y1, y2, y3])
        z = np.concatenate([z1, z2, z3])

        #traj = self.interpolate_points(x, y, z)
        traj = np.vstack([x, y, z])


        return traj
    

    def medical_incision_curve_3d(self):
        """
        Curve which represents a medical incision including curve movement after the incision.

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        
        
        # Parameter
        radius_1 = 0.2    
        radius_2 = 0.05        # radios of the semi circle    

        r3 = 0.1

        segment_length = 0.15     # length of straight line

        k12 = 0.4   # transition factor from segment 1 to segment 2
        k23 = 1

        k34 = 1

        tol = 1e-2     # tolerance for numerical errors at transition into new segment

        # circle
        t1 = np.linspace(0, 1, int(self.num_points * 0.4))
        x1 = t1 * segment_length
        z1 = radius_1 + radius_1 * np.sin(np.pi * 1.1 + t1 * 0.32 * np.pi)
        y1 = np.zeros_like(x1)


        # Straight
        t2 = np.linspace(0 + tol, 1, int(self.num_points * 0.1))                # use values and derivative of last section to calculate the straight line
        x2 = x1[-1] + t2 * segment_length * k12
        y2 = np.zeros_like(x2)

        deriv_z1_e = radius_1 * 0.32 * np.pi * np.cos(np.pi *1.1 + t1[-1] * 0.32 * np.pi)     # calc derivative of z at end of first segment
        z2 = z1[-1] + deriv_z1_e * t2 * k12

        # #derivs after segment2
        # x_der = segment_length * k12
        # y_der = 0
        # z_der = deriv_z1_e * k12
        # print(f'derivs after segment2 = {x_der, y_der, z_der}')
              

        # circle
        t3 = np.linspace(0 + tol, 1, int(self.num_points * 0.3))    # angle of circle

        x3 = x2[-1] + t3 * segment_length * k23 
        y3 = y2[-1] + t3 * 0
        v0 = -0.33732
        v = deriv_z1_e  / (k23 * r3*np.pi*np.cos(v0*np.pi)) #-0.2575 

        z3 = z2[-1] + r3 * np.sin(v0 * np.pi + t3 * np.pi * v) - r3 * np.sin(v0 * np.pi)

        # # deriv last point
        x_der = segment_length * k23
        y_der = 0
        z_der = r3 * np.cos(v0 * np.pi + 1 * np.pi * v) * v * np.pi * k23
        print(f'derivs after segment3 = {x_der, y_der, z_der}')


        ### circle
        k4 = 0.6
    
        t4 = np.linspace(0 + tol, 1, int(self.num_points * 0.2))    # angle of circle

        x4 = x3[-1] + np.sin(t4 * np.pi * 0.7) * 0.1 * 0.682 * k4
        y4 = y3[-1] + (- np.cos(t4 * np.pi * 0.7) * 0.1 + 0.1) * k4
        z4 = z3[-1] + t4 * 0.05001 * k4

        # # derivs start 4
        x_der = np.cos(t4[0] * np.pi * 0.7) * 0.1 * np.pi * 0.7 * 0.682
        y_der = np.sin(t4[0] * np.pi * 0.7) * 0.1 * np.pi * 0.7
        z_der = 0.05001
        print(f'derivs after bevore seg 4 = {x_der, y_der, z_der}')

        x = np.concatenate([x1, x2, x3, x4])
        y = np.concatenate([y1, y2, y3, y4])
        z = np.concatenate([z1, z2, z3, z4])


        return np.vstack((x, y, z))
    

    def rehab(self):
        """
        Generate a practical oriented trajectory for the reference path. Possible shoulder rehab movement. 
        Straight line -> movement upwards -> straight line -> movement downwards.
        """

        # Parameter
        length = 0.2
        height = 0.1

        tol = 1e-6

        x1 = np.linspace(0, length, self.num_points // 5)
        y1 = np.zeros_like(x1)
        z1 = np.zeros_like(x1)

        x2 = np.linspace(length+tol, length * 2, self.num_points // 5)
        y2 = np.zeros_like(x2)
        t2 = np.linspace(-1/4, 1/4, self.num_points // 5)
        z2 = height/2 * np.sin(2*np.pi*t2) + height/2

        x3 = np.linspace(length*2 + tol, length*3, self.num_points // 5)
        y3 = np.zeros_like(x3)
        z3 = np.full_like(x3, height)

        x4 = np.linspace(length*3 + tol, length*4, self.num_points // 5)
        y4 = np.zeros_like(x4)
        t4 = np.linspace(1/4, 3/4, self.num_points // 5)
        z4 = height/4 * np.sin(2*np.pi*t4) + height*3/4

        x5 = np.linspace(length*4 + tol, length*5, self.num_points // 5)
        y5 = np.zeros_like(x5)
        z5 = np.ones_like(x5) * height/2

        x = np.concatenate([x1, x2, x3, x4, x5])
        y = np.concatenate([y1, y2, y3, y4, y5])
        z = np.concatenate([z1, z2, z3, z4, z5])

        traj = np.vstack([x, y, z])


        return traj
    

    def rehab_v2(self):
        """
        Generate a practical oriented trajectory for the reference path. Possible shoulder rehab movement.
        Straight line -> semi circle -> straight line. Include smooth transitions between the segments.
        """
        # Parameter
        length = 0.2
        height = 0.1

        l_y = 0.1

        tol = 1e-6

        x1 = np.linspace(0, length, self.num_points // 5)
        y1 = np.linspace(0, l_y, self.num_points // 5)
        z1 = np.zeros_like(x1)

        x2 = np.linspace(length+tol, length * 1.5, self.num_points // 10)
        y2 = np.linspace(l_y+tol, l_y * 1.5, self.num_points // 10)
        t2 = np.linspace(-1/4, 0, self.num_points // 10)
        z2 = height/2 * np.sin(2*np.pi*t2) + height/2

        x3 = np.linspace(length*1.5 + tol, length*3.5, self.num_points // 5 * 2)
        y3 = np.linspace(1.5* l_y + tol, l_y * 3.5, self.num_points // 5 * 2)
        t3 = np.linspace(0, 1/2, self.num_points // 5 * 2)
        z3 = height/2 * np.sin(2*np.pi*t3) + height/2

        x4 = np.linspace(length*3.5 + tol, length*4, self.num_points // 10)
        y4 = np.linspace(l_y*3.5 + tol, l_y*4, self.num_points // 10)
        t4 = np.linspace(1/2, 3/4, self.num_points // 10)
        z4 = height/2 * np.sin(2*np.pi*t4) + height/2

        x5 = np.linspace(length*4 + tol, length*5, self.num_points // 5)
        y5 = np.linspace(l_y*4 + tol, l_y*5, self.num_points // 5)
        z5 = np.zeros_like(x5)

        x = np.concatenate([x1, x2, x3, x4, x5])
        y = np.concatenate([y1, y2, y3, y4, y5])
        z = np.concatenate([z1, z2, z3, z4, z5])

        traj = np.vstack([x, y, z])


        return traj

    def rehab_3D(self):
        """
        Generate a practical oriented trajectory for the reference path. Possible shoulder rehab movement. Include movement in all 3 dimensions.
        Straight line -> movement upwards -> straight line -> movement downwards.
        """

        # Parameter
        length = 0.14
        height = 0.1

        tol = 1e-6

        x1 = np.linspace(0, length, self.num_points // 5)
        y1 = x1
        z1 = np.zeros_like(x1)

        x2 = np.linspace(length+tol, length * 2, self.num_points // 5)
        y2 = x2
        t2 = np.linspace(-1/4, 1/4, self.num_points // 5)
        z2 = height/2 * np.sin(2*np.pi*t2) + height/2

        x3 = np.linspace(length*2 + tol, length*3, self.num_points // 5)
        y3 = x3
        z3 = np.full_like(x3, height)

        x4 = np.linspace(length*3 + tol, length*4, self.num_points // 5)
        y4 = x4
        t4 = np.linspace(1/4, 3/4, self.num_points // 5)
        z4 = height/4 * np.sin(2*np.pi*t4) + height*3/4

        x5 = np.linspace(length*4 + tol, length*5, self.num_points // 5)
        y5 = x5
        z5 = np.ones_like(x5) * height/2

        x = np.concatenate([x1, x2, x3, x4, x5])
        y = np.concatenate([y1, y2, y3, y4, y5])
        z = np.concatenate([z1, z2, z3, z4, z5])

        traj = np.vstack([x, y, z])


        return traj
    

    def rehab_3D_curve(self):
        """
        Generate a practical oriented trajectory for the reference path. Possible shoulder rehab movement. Include movement in all 3 dimensions.
        Straight line -> movement upwards -> straight line -> movement downwards.
        """

        # Parameter
        length = 0.2
        height = 0.1

        tol = 1e-6

        n1 = int(self.num_points*0.22)
        x1 = np.linspace(0, length, n1)
        y1 = np.zeros_like(x1)
        z1 = np.zeros_like(x1)

        n2 = int(self.num_points * 0.22)
        x2 = np.linspace(length+tol, length * 2, n2)
        y2 = np.zeros_like(x2)
        t2 = np.linspace(-1/4, 1/4, n2)
        z2 = height/2 * np.sin(2*np.pi*t2) + height/2

        n3 = int(self.num_points * 0.1)
        x3 = np.linspace(length*2 + tol, length*2.5, n3)
        y3 = np.zeros_like(x3)
        z3 = np.full_like(x3, height)

        n4 = int(self.num_points * 0.23)
        t4 = np.linspace(0 + tol, 1/4, n4)
        x4 = x3[-1] + np.sin(t4 * np.pi) * 0.3
        y4 = (-np.cos(t4 * np.pi) - np.cos(np.pi))*0.3
        z4 = np.full_like(x4, height)

        n5 = int(self.num_points * 0.23)
        t5 = np.linspace(1/4, 3/4, n5)
        x5 = np.linspace(x4[-1] + tol, x4[-1] + length, n5)
        y5 = np.linspace(y4[-1] + tol, y4[-1] + length, n5)
        z5 = height/4 * np.sin(2*np.pi*t5) + height*3/4

        x = np.concatenate([x1, x2, x3, x4, x5])
        y = np.concatenate([y1, y2, y3, y4, y5])
        z = np.concatenate([z1, z2, z3, z4, z5])

        traj = np.vstack([x, y, z])


        return traj


    def circle(self):
        """
        Generate a practical oriented trajectory for the reference path. Circle followed by a straight segment. 

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        # Parameter
        radius = 0.3            # radios of the semi circle
        center_x = 0        
        center_y = 0        
        center_z = 0.3        
        line_length = 0.5      # length of straight line

        theta = np.linspace(0, np.pi, self.num_points)       # angle of circle
        #theta = np.linspace(np.pi/6*5, np.pi, num_points//6*5)       # angle of circle

        # Circle
        z_circle = center_z + radius * np.cos(theta)        
        x_circle = center_x + -1*radius * np.sin(theta)     
        y_circle = np.full_like(x_circle, center_y)    

        #traj = self.interpolate_points(x, y, z)
        traj = np.vstack([x_circle, y_circle, z_circle])


        return traj
    

    def medical_incision_curve_3D(self):
        """
        Generate a practical oriented trajectory for the reference path. Circle followed by a straight segment. 
        Also add a linear change in y direction.

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        # Parameter
        radius = 0.1            # radios of the semi circle
        center_x = 0        
        center_y = 0        
        center_z = 0.1        
        line_length = 0.15      # length of straight line

        theta = np.linspace(0, np.pi, self.num_points // 2)       # angle of circle
        #theta = np.linspace(np.pi/6*5, np.pi, num_points//6*5)       # angle of circle

        # Circle
        z_circle = center_z + radius * np.cos(theta)        
        x_circle = center_x + -1*radius * np.sin(theta)     
        #y_circle = np.full_like(x_circle, center_y)   
        y_circle = np.linspace(0, 1, self.num_points // 2) 
              
        # Line
        x_line = np.linspace(0.0000001, line_length, self.num_points//2)
        #y_line = np.zeros_like(x_line)
        y_line = np.linspace(line_length + 0.00000001, line_length * 2, self.num_points//2)
        z_line = np.zeros_like(x_line)

        x = np.concatenate([x_circle, x_line])
        y = np.concatenate([y_circle, y_line])
        z = np.concatenate([z_circle, z_line])

        #traj = self.interpolate_points(x, y, z)
        traj = np.vstack([x, y, z])

    
        return traj
    

    def lines(self):
        """
        Generate multiple straight line for the reference trajectory. 

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        x1 = np.linspace(0, 0.9999, self.num_points // 3)
        y1 = np.zeros_like(x1)
        z1 = np.zeros_like(x1)

        y2 = np.linspace(0, 0.9999, self.num_points // 3)
        x2 = np.ones_like(y2)
        z2 = np.zeros_like(y2)

        x3 = np.linspace(1.00001, 0, self.num_points // 3)
        y3 = np.linspace(1, 2, self.num_points // 3)
        z3 = np.linspace(0, 0.5, self.num_points // 3)

        x = np.concatenate([x1, x2, x3])
        y = np.concatenate([y1, y2, y3])
        z = np.concatenate([z1, z2, z3])

        traj = np.vstack([x, y, z])

        return traj
    

    def helix(self):
        '''
        Generate a helix for the reference trajectory. 
        '''
        radius = 0.1
        rotations = 2
        hight_factor = 0.1

        s = np.linspace(0, rotations, self.num_points)
        x = radius * np.cos(2*np.pi*s)
        y = radius*np.sin(2*np.pi*s)
        z = hight_factor*s

        x -= x[0]
        y -= y[0]
        z -= z[0]

        return np.vstack([x, y, z])


    # def helix(self):          # test -> calculation of kappa and tau also works for non uniform s
    #     """
    #     Generate a helix trajectory with variable spacing using random step sizes.

    #     Parameters:
    #         min_step (float): minimum random step in s
    #         max_step (float): maximum random step in s

    #     Returns:
    #         traj: 3 x N array with helix coordinates
    #         s: 1D array of accumulated arc-length parameter
    #     """
    #     min_step=0.000005, 
    #     max_step=0.9
    #     radius = 0.1
    #     rotations = 2
    #     hight_factor = 0.1

    #     # 1. Erzeuge zufällige Schrittweiten
    #     random_steps = np.random.uniform(min_step, max_step, size=self.num_points - 1)

    #     # 2. Kumulierte Summe → erzeugt nichtlineares s
    #     s = np.insert(np.cumsum(random_steps), 0, 0.0)

    #     # 3. Skaliere auf gewünschte max. Länge (rotations)
    #     s = s / s[-1] * rotations

    #     # 4. Berechne Helix-Koordinaten
    #     x = radius * np.cos(2 * np.pi * s)
    #     y = radius * np.sin(2 * np.pi * s)
    #     z = hight_factor * s

    #     traj = np.vstack((x, y, z))
    #     return traj


    def sin_curve(self):
        '''
        Generate an Sin-curve
        '''
        x = np.linspace(0, 1/3, self.num_points)
        y = np.zeros_like(x)
        z = np.sin(12*np.pi*x)*0.05

        traj = np.vstack([x, y, z])

        
        return traj
    

    def practical(self):
        """
        Curve which represents a medical incision including curve movement after the incision.

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        
        # Parameter
        radius = 0.1            # radios of the semi circle
        center_x = 0        
        center_y = 0        
        center_z = 0.1        
        line_length = 0.25      # length of straight line

        theta = np.linspace(0, np.pi, self.num_points // 2)       # angle of circle
        #theta = np.linspace(np.pi/6*5, np.pi, num_points//6*5)       # angle of circle

        # Circle
        z_circle = center_z + radius * np.cos(theta)        
        x_circle = center_x + -1*radius * np.sin(theta)     
        y_circle = np.full_like(x_circle, center_y)    
              
        # Line
        x_line = np.linspace(0.0000001, line_length, self.num_points//2) #0.0000001
        y_line = np.zeros_like(x_line)
        z_line = np.zeros_like(x_line)

        x = np.concatenate([x_circle, x_line])
        y = np.concatenate([y_circle, y_line])
        z = np.concatenate([z_circle, z_line])

        #traj = self.interpolate_points(x, y, z)
        traj = np.vstack([x, y, z])


        return traj
    

    def standard_line(self):
        """
        Generate a standard line for the reference trajectory. 

        Output:
            - x, y, z: coordinates of the points on the reference path [(num_points,) numpy array]
        """
        y = np.linspace(0, 0.55, self.num_points)
        x = np.zeros_like(y)
        z = np.zeros_like(y)

        traj = np.vstack([x, y, z])

        return traj