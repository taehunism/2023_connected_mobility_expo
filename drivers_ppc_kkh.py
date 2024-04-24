import numpy as np
import math
import time
class PurePursuitDriver:


    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance
        self.wheelbase = 0.21

    def find_lookahead_idx(self, pose_x, pose_y, ref):
        start = time.time()

        #find the lookahead index directly in front of the car
        distances = np.sqrt((ref[:, 0]) ** 2 + (ref[:, 1]) ** 2)
        #print(distances[0:6])
        #distances = np.sqrt((ref[:, 0] - pose_x) ** 2 + (ref[:, 1] - pose_y) ** 2)
        #min_idx = np.argmin(distances)       ##find index of minimum value
        min_idx = np.argmin(distances)       ##too slow function fucking 
        #print(min_idx)
        #lookahead_idx = min_idx      
        lookahead_idx = min_idx
        #print('min_idx: ',min_idx)
        #print('x of min_idx: ',ref[min_idx,0])
        #print('y of min_idx: ',ref[min_idx,1])

        while True:
            lookahead_idx += 1
            if lookahead_idx >= len(ref):
                lookahead_idx = 0
            #dist_to_lookahead = np.hypot(ref[lookahead_idx, 0] - pose_x, ref[lookahead_idx, 1] - pose_y)  ##calculate distance between robot pose with ref of lookahead
            dist_to_lookahead = np.hypot(ref[lookahead_idx, 0], ref[lookahead_idx, 1])
            #dist_to_lookahead = np.sqrt(ref[lookahead_idx, 0]**2 + )
            if dist_to_lookahead >= self.lookahead_distance:   ##if waypoint is farther than the lookahead distance
                break
        
        #if (lookahead_idx > 140 or lookahead_idx < 24) and self.count==1:
            #self.count=1
            #self.lookahead_distance = 0.7
            #return find_lookahead_idx(pose_x, pose_y, ref)      ##recursive function
            
        #self.count=0
        #print('lookahead_idx: ',lookahead_idx)
        #print('time', time.time()-start)

        #rint('x of lookahead_idx: ',ref[lookahead_idx,0])
        #print('y of lookahead_idx: ',ref[lookahead_idx,1])

        return lookahead_idx

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        # Find the lookahead point on the reference trajectory
        lookahead_idx = self.find_lookahead_idx(pose_x, pose_y, ref)
        lookahead_point = ref[lookahead_idx]             ##P_l

        # Compute the heading to the lookahead point
        #print('lookahead_point[0]: ',lookahead_point[0])
        #print('lookahead_point[1]: ',lookahead_point[1])
        
        #pure pursuit 
        #a = math.atan2(lookahead_point[1],lookahead_point[0])
        #target_angle = 2 * a 
        
        target_angle = math.atan2(lookahead_point[1], lookahead_point[0])
        print('local_lookahead[0]: ',lookahead_point[0])
        print('local_lookahead[1]: ',lookahead_point[1])
        taget_angle = math.atan2(lookahead_point[1] - pose_y, lookahead_point[0] - pose_x)   
        #print('target angle: ',target_angle)
        #print('pose_theta :',pose_theta)
        alpha = target_angle - pose_theta      ##Look ahead heading(theta)
        #print('alpha: ',alpha)
        #alpha = target_angle
        #print('Look ahead heading(theta)1: ',alpha)

        # Ensure the angle is within the range of [-pi, pi]
        
        while alpha > np.pi:
            alpha -=  2 * np.pi
        while alpha < -np.pi:
            alpha +=  2 * np.pi
        
        #print('Look ahead heading(theta)2: ',alpha)
        # Compute the steering angle
        steering_angle = (math.atan2(2 * self.wheelbase * math.sin(target_angle), self.lookahead_distance))
        #steering_angle = -1*steering_angle 

        #steering_angle = (math.atan2(2 * math.sin(alpha), self.lookahead_distance)
        #)*0.7                    ##maybe 0.7 is scaling factor
        #if abs(steering_angle) < 0.3:
        #    steering_angle = steering_angle*0.3
        print('steering angle: ',steering_angle)
        print('------------------------------------')
        # Compute the speed (you may use constant speed simply)

        # speed = (0.1*9.81*0.35/np.sin(abs(steering_angle)))**0.5
        speed = 0.15
        #print(speed)
        return speed, steering_angle, lookahead_idx

