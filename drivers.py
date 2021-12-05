
import numpy as np

import pandas as pd

import LQR_control_6 as LQR
import cubic_spline_planner as csp



# LQR parameter
#lqr_Q = np.eye(5)
lqr_Q = [[7,0,0,0,0],
        [0,0.13,0,0,0],
        [0,0,3.15,0,0],
        [0,0,0,0.82,0],
        [0,0,0,0,9]] 
#6.5,0.1,3.13,0.8, 7, 111.44

#lqr_R = np.eye(2)
lqr_R = [[1,0],
        [0,0.7]]
#1, 0.135, 111.44
dt = 0.1  # time tick[s]
L = 3.0  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad] 45.0


#global
data = pd.read_csv('./maps/Sochi_centerline.csv', sep=", ")
data_xy = data[["x_m","y_m"]]
cx = data_xy["x_m"]
cy = data_xy["y_m"]

cx1, cy1, cyaw1, ck1, s1 = csp.calc_spline_course(cx, cy, ds=0.1)

#cyaw1 modification
#success
#0.012, 0.022, 0.032,0.042  111.44
for i in range(len(ck1)):
    if abs(ck1[i]) >= 0.2 and abs(ck1[i]) < 0.3:
        cyaw1[i] += np.sign(ck1[i])*0.02
    
    elif abs(ck1[i]) >= 0.3 and abs(ck1[i]) < 0.5:
        cyaw1[i] += np.sign(ck1[i])*0.035

    elif abs(ck1[i]) >= 0.5 and abs(ck1[i]) < 0.6:
        cyaw1[i] += np.sign(ck1[i])*0.04

    elif abs(ck1[i]) >= 0.6:
        cyaw1[i] += np.sign(ck1[i])*0.045

    if abs(ck1[i]) >= np.pi:
        cyaw1[i] -= np.sign(cyaw1)*np.pi    

#basic 15/3.6
target_speed = 52/3.6 # [m/s] 
#47, 112.00

flag = 0
sp = LQR.calc_speed_profile(cyaw1, target_speed, flag)

# plt.plot(cyaw1)
# plt.plot(ck1)
# plt.show()

# the number of LiDAR points
NUM_RANGES = 1080
# angle between each LiDAR point
ANGLE_BETWEEN = 2 * np.pi / NUM_RANGES
# number of points in each quadrant
NUM_PER_QUADRANT = NUM_RANGES // 4

min_acc = 5.5 #[m/s^2]
#4, 127.72
#4.5 126.82
#5, 112.54

# Lfc = 2
# k = 1

# drives straight ahead at a speed of 5
class SimpleDriver:

    def process_lidar(self, ranges, poses):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle


# drives toward the furthest point it sees
class AnotherDriver:

    def __init__(self):
            self.e = 0.0    #lateral distance to the path
            self.th_e = 0.0 #angle different to the path
            
            self.state = LQR.State(x=0, y=0, yaw=0, v=0)
            self.dl =0

            #self.rear_x = 0 
            #self.rear_y = 0 

    def process_lidar(self, ranges, poses):

        max_idx = np.argmax(ranges[NUM_PER_QUADRANT:-NUM_PER_QUADRANT]) + NUM_PER_QUADRANT
        max_distance = ranges[max_idx]
        #steering_angle = max_idx * ANGLE_BETWEEN - (NUM_RANGES // 2) * ANGLE_BETWEEN
        TTC = max_distance / (self.state.v *np.cos(self.dl))
        bound = self.state.v / min_acc
 
        self.state.x = poses[0][0]
        self.state.y = poses[1][0]
        self.state.yaw = poses[2][0]

        #self.rear_x = self.state.x - ((L / 2) * math.cos(self.state.yaw))
        #self.rear_y = self.state.y - ((L / 2) * math.sin(self.state.yaw))
        
        self.dl, _, self.e, self.th_e, ai, ind = LQR.lqr_speed_steering_control(self.state, cx1, cy1, cyaw1, ck1, self.e, self.th_e, sp, lqr_Q, lqr_R)

        self.state.v += ai*dt

        # if self.th_e >= 0.2:
        #     if ind < len(cx1):
        #         tx = cx1[ind]
        #         ty = cy1[ind]
        #     else:  # toward goal
        #         tx = cx1[-1]
        #         ty = cy1[-1]
        #         ind = len(cx1) - 1

        #     Lf = k * self.state.v + Lfc  # update look ahead distance    

        #     alpha = math.atan2(ty - self.rear_y, tx - self.rear_x) - self.state.yaw

        #     self.dl = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

        if TTC <bound:
            self.state.v -= 2.5*min_acc*dt
            self.dl += np.sign(self.dl)*0.04

            return self.state.v, self.dl

        if abs(self.e) >= 0.4:
            self.dl += np.sign(self.dl)*0.01

        elif abs(self.e) >= 0.5:
             self.dl += np.sign(self.dl)*0.013

        elif abs(self.e) >=0.7:
            self.dl += np.sign(self.dl)*0.015

        # if abs(ck1[ind]) >= 0.4:  
        #     self.dl += np.sign(self.dl)*0.01

        # elif abs(ck1[ind]) >= 0.5:
        #      self.dl += np.sign(self.dl)*0.02

        # elif abs(ck1[ind]) >=0.7:
        #     self.dl += np.sign(self.dl)*0.025
              

        
        if abs(self.dl) >= 0.5:
            self.state.v = 12/3.6
        
        elif abs(self.dl) >= 0.35:
            self.state.v = 16/3.6 #[m/s] 

        elif abs(self.dl) >= 0.25:
            self.state.v = 18.5/3.6 #[m/s] 

        elif abs(self.dl) >= 0.1:
            self.state.v = 20.5/3.6 # [m/s] 

        elif abs(self.dl) >= 0.05:
            self.state.v = 23/3.6 # [m/s]
        
  
        print("theta error: ", self.th_e)
        print("error: ",self.e)
        print("speed: ",self.state.v)
        print("steering: ", self.dl)

        # plt.plot(np.diff(cyaw1))
        # plt.plot(sp)
        # plt.show()     
        
    
        return self.state.v, self.dl

        