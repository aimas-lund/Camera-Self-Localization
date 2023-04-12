import numpy as np
from numpy.linalg import inv

def get_transformation_matrices(th):
    #     R = [            cy*cz,           -cy*sz,     sy]
    #         [ cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx]
    #         [ sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]
    #       = Rx(tx) * Ry(ty) * Rz(tz)

    cos = np.cos
    sin = np.sin
    tan = np.tan

    # Calculate all of this ones (more so it takes less space in the code)
    cx = cos(th[0, 0])
    cy = cos(th[1, 0])
    cz = cos(th[2, 0])
    
    sx = sin(th[0, 0])
    sy = sin(th[1, 0])
    sz = sin(th[2, 0])
    
    ty = tan(th[1, 0])

    R = np.matrix([[cy*cz, -cy*sz, sy],
                   [cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx],
                   [sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]])
    
    # angle_dot = W_inv * angular_velocity
    # eta_dot = W_inv * v
    W_inv = np.matrix([[1, sx*ty, cx*ty],
                       [0, cx,   -sy],
                       [0, sx/cy, cx/cy]])
    W = np.matrix([[1,  0, -sy],
                   [0,  cx, cy*sx],
                   [0, -sx, cy*cx]])
    
    return R, W_inv

class kalman_filter():
    def __init__(self):
        class parameters:
            def __init__(self):
                self.k = 0.01
                self.m = 2.4  # m100
                self.g = -9.81  # The gravitational constant
                self.L = 0.225
                self.b = 0.001
                self.ts = None  # Sampling time
                self.D = np.eye(3) * 0.01
                self.I = np.matrix([[0.000003, 0.0, 0.0],
                                    [0.0, 0.000003, 0.0],
                                    [0.0, 0.0, 0.00001]])
                
                self.Q = np.eye(6) * 1

                self.R = np.eye(3) * 10000
                
                A = np.matrix([[1, 0, 0, self.ts, 0, 0],
                               [0, 1, 0, 0, self.ts, 0],
                               [0, 0, 1, 0, 0, self.ts],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 1]])
                self.A = np.zeros((12,12))
                # Fill in the update data
                self.A[0:6,0:6] = A
                self.A[6:12,6:12] = A
                self.A = A

                self.H = np.zeros((6,12))
                self.H[0:3,0:3] = np.eye(3)
                self.H[3:6,6:9] = np.eye(3)
                self.H = np.zeros((3,6))
                self.H[0:3,0:3] = np.eye(3)
                
                # Controller parameters
                self.kp_v = 0.1
                self.ki_v = 0.001
                self.kd_v = 0.005

                self.kp_th_v = 1#10
                self.ki_th_v = 10#100
                self.kd_th_v = 0.001#0.001

                self.kp_th = 20
                self.kd_th = 40

                self.kp_z_v = 20
                self.ki_z_v = 15
                self.kd_z_v = 0.006

                # Integrators and old err values
                self.int_v_x_err = 0
                self.int_v_y_err = 0
                self.old_v_x_err = None
                self.old_v_y_err = None

                self.int_v_th_err = 0
                self.old_v_th_err = None

                self.int_v_z_err = 0
                self.old_v_z_err = None

                self.control_A = np.matrix(
                    [[self.L * self.k, 0, -self.L * self.k, 0],
                    [0, self.L * self.k, 0, -self.L * self.k],
                    [self.b, -self.b, self.b, -self.b],
                    [1, 1, 1, 1]])

            def set_ts(self, ts):
                self.ts = ts
                self.A = np.matrix([[1, 0, 0, ts, 0, 0],
                                    [0, 1, 0, 0, ts, 0],
                                    [0, 0, 1, 0, 0, ts],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]])

        self.param = parameters()
        self.x = np.zeros(12).reshape(12,1)
        self.x_kalman = np.zeros(12).reshape(12,1)
        self.u = np.zeros(4).reshape(4,1)
        self.z = np.zeros(3).reshape(3,1) # Measurement

        self.p = np.zeros((6,6))
        self.old_time = None
        
    def update_loop(self, inputs, dist_x, dist_y, drone_height, time):
        # Skip the first itteration so that ts can be gotten
        if self.old_time is None:
            self.old_time = time
            return None, None
        # Inputs
        self.u[0] = inputs[0]
        self.u[1] = inputs[1]
        self.u[2] = inputs[2]
        self.u[3] = inputs[3]
        
        # Measurement
        self.z[0] = dist_x
        self.z[1] = dist_y
        self.z[2] = drone_height
        
        ts = time - self.old_time  #TODO NBNBNB This should match with the kalman somehow, there might need to be two ts
        self.param.set_ts(ts)
        # kalman_param.ts = ts

        u_drone = self.controller_simulation()
        x_predict = self.sim_step(u_drone)
        
        # TODO an if statement here so that the kalman filter only updates on new data
        x_small = self.x[:6].copy()
        x_small[3] = self.x[6]
        x_small[4] = self.x[7]
        x_small[5] = self.x[8]
        x_predict = self.kalman_filter(x_small)
        
        for j in range(3):
            self.x[j] = x_predict[j]
        
        return self.x[0, 0], self.x[1, 0]
        
        
    def controller_simulation(self):
        # Split the input into its values
        v_ref = self.u[0:3].reshape(3,1)
        rz_ref = self.u[3]
        # Split the state vector
        th = self.x[3:6].reshape(3,1)
        v = self.x[6:9].reshape(3,1)
        w = self.x[9:12].reshape(3,1)
        # Get transformation matrices
        R, W_inv = get_transformation_matrices(th)
        # Calucate the euler angle rates
        d_th = W_inv.dot(w)
        
        # Pp error
        v_x_err = v_ref[0] - v[0]
        v_y_err = v_ref[1] - v[1]
        # Ip error
        self.param.int_v_x_err = self.param.int_v_x_err + self.param.ts * v_x_err
        self.param.int_v_y_err = self.param.int_v_y_err + self.param.ts * v_y_err
        # Dp error
        if self.param.old_v_x_err:
            d_v_x_err = (v_x_err - self.param.old_v_x_err) / self.param.ts
            d_v_y_err = (v_y_err - self.param.old_v_y_err) / self.param.ts
        else:
            d_v_x_err = 0
            d_v_y_err = 0
        # Update old error variable
        self.param.old_v_x_err = v_x_err
        self.param.old_v_y_err = v_y_err
        # Calculate the result from the controller
        v_x_ref = self.param.kp_v * v_x_err +\
                  self.param.ki_v * self.param.int_v_x_err +\
                  self.param.kd_v * d_v_x_err
        v_y_ref = self.param.kp_v * v_y_err +\
                  self.param.ki_v * self.param.int_v_y_err +\
                  self.param.kd_v * d_v_y_err
        v_xy_ref = np.zeros(3).reshape(3,1)
        v_xy_ref[0] = v_x_ref
        v_xy_ref[1] = v_y_ref
        
        #print(v_x_err, param.int_v_x_err, d_v_x_err, v_x_ref)

        # Pth error
        v_th_err = rz_ref - d_th[2]
        # Ith error
        self.param.int_v_th_err = self.param.int_v_th_err + v_th_err * self.param.ts
        # Dth error
        if self.param.old_v_th_err:
            d_v_th_err = (v_th_err - self.param.old_v_th_err) / self.param.ts
        else:
            d_v_th_err = 0
        # Update the old error
        self.param.old_v_th_err = v_th_err
        # Calculate the result from the controller
        v_th_ref = self.param.kp_th_v * v_th_err +\
                   self.param.ki_th_v * self.param.int_v_th_err +\
                   self.param.kd_th_v * d_v_th_err

        # Use the controller results to create a th reference
        th_ref = np.zeros(3).reshape(3,1)
        th_ref[2] = v_th_ref
        # Add the result from the xy speed controllers
        tmp = np.matrix([[0, -1, 0],
                        [1, 0, 0]])
        th_ref[:2] = tmp.dot(R).dot(v_xy_ref)
        
        # Calculate the errors
        th_err = th_ref - th
        d_th_err = th_err - d_th
        # Calculate the result
        uu = self.param.kp_th * th_err +\
             self.param.kd_th * d_th_err
        
        # Caluclate the z v errors
        v_z_err = v_ref[2] - v[2]
        self.param.int_v_z_err = self.param.int_v_z_err + self.param.ts * v_z_err
        if self.param.old_v_z_err:
            d_v_z_err = (v_z_err - self.param.old_v_z_err) / self.param.ts
        else:
            d_v_z_err = 0
        self.param.old_v_z_err = v_z_err

        # Calculate the result from the controller
        f_z = self.param.m *\
            (self.param.kp_z_v * v_z_err +\
            self.param.ki_z_v * self.param.int_v_z_err +\
            self.param.kd_z_v * d_v_z_err)
        
        B = np.zeros(4).reshape(4,1)
        B[0] = self.param.I[0,0] * uu[0]
        B[1] = self.param.I[1,1] * uu[1]
        B[2] = self.param.I[2,2] * uu[2]
        B[3] = (f_z - self.param.m * self.param.g)/(self.param.k*np.cos(th[0])*np.cos(th[1]))
        
        gamma = inv(self.param.control_A).dot(B)
        inputs = np.zeros(4).reshape(4,1)
        # Gamma cannot be negative
        for i, g in enumerate(gamma):
            if g < 0:
                g = 0
            else:
                g = np.sqrt(g)
            inputs[i] = g

        return inputs

    def sim_step(self, u):
        x_next = self.x + self.param.ts * self.dynamics(u)
        return x_next

    def dynamics(self, u):
        # Split out the states
        p = self.x[0:3].reshape(3,1), 
        th = self.x[3:6]
        v = self.x[6:9].reshape(3,1)
        w = self.x[9:12].reshape(3,1)

        # Calculate the force from the propellers
        R, W_inv = get_transformation_matrices(th)
        FB = np.zeros(3).reshape(3,1)
        FB[2] = self.param.k * sum(np.power(u,2))
        if FB[2] < 0:
            FB[2] = 0
        
        # Calculate the resistance from the air
        FD = self.param.D.dot(v)
        
        # Calculate the force from gravity
        FG = np.array([0, 0, self.param.m * self.param.g]).reshape(3,1)

        # Get the total force and use it to get the acceleration
        dv = 1/self.param.m * (inv(R) * FG + FB - FD)
        dp = R * v
        
        tau_B = np.array([self.param.L * self.param.k * (u[0]**2 - u[2] **2),
                          self.param.L * self.param.k * (u[1]**2 - u[3] **2),
                          self.param.b * (u[0]**2 - u[1]**2 + u[2]**2 - u[3]**2)]).reshape(3,1)
        
        cross = np.cross(w.reshape(1,3), (self.param.I * w).reshape(1,3)).reshape(3,1)
        dw = inv(self.param.I).dot(tau_B - cross)
        # Transform from angular speed to change in theta
        dth = W_inv.dot(w)
        
        return np.append(np.append(np.append(dp, dth, axis=0), dv, axis=0), dw, axis=0)
    
    def kalman_filter(self, x_predict):
        # Made by following: https://campar.in.tum.de/Chair/KalmanFilter
        p_new = self.get_covariance()
        k = self.compute_kalman_gain(p_new)
        x_corrected = self.update_state_estimate(x_predict, k)
        self.update_covariance(p_new, k)

        return x_corrected

    def get_covariance(self):
        # P_new = A*P_old*A^T + Q
        p_new = self.param.A.dot(self.p.dot(self.param.A.transpose())) + self.param.Q
        return p_new
        
    def compute_kalman_gain(self, p):
        # K = P_new*H^T*(H*P_new*H^t + R)^-1
        k = p.dot(self.param.H.transpose()).dot(inv(self.param.H.dot(p).dot(self.param.H.transpose()) + self.param.R))
        # Pretty nasty to write this in python, but not sure what else to do
        # Trust the comment above
        return k

    def update_state_estimate(self, x_predict, k):
        # x_corrected = x_predicted + K*(z - H*x_predicted)
        x_corrected = x_predict + k.dot(self.z - self.param.H.dot(x_predict))
        
        return x_corrected

    def update_covariance(self, p, k):
        # P_new = (I - K*H)*P_old
        self.p = (np.eye(p.shape[0]) - k.dot(self.param.H)).dot(p)
