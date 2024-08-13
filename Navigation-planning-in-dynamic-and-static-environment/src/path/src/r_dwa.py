"""parameters
最大线速度3m/s 最大线加速度3m/s^2
最大角速度5rad/s 最大角加速度5rad/s^2
"""
import math
from action import Action
import time
import sys
import rospy
#调试
#from debug import Debugger
from LaserData import AMCLLaserData
class Robot(object):
    def __init__(self):
        self.robot_size = 8 # 机器人参数 200
        self.avoid_dist = 8
        self.StepLength = 2 # 机器人执行能力
        self.TimeInterval = 1e-1 # 机器人控制时间间隔
        self.PredictTime = 1 # 选取(v,w)时机器人路径的预测时间
        self.maxVel = 0.1 # 机器人最大线速度 3000
        self.maxAddVel = 1 # 机器人最大线加速度 3000
        self.maxW = 0.314 # 机器人最大角速度 5.0
        self.maxAddW = 3.14 # 机器人最大角加速度 5.0
        self.ToGoalCostDis = 2000 # 评价函数中朝向目标点的权重
        self.ToGoalCostAngle = 200 # 评价函数中朝向目标角度的权重
        self.ToGoalCostV = 0 # here needs to be changed
        self.ToGoalCostW = 0 # here needs to be changed
        self.ToGoalCostFinalAngle = 0 # here needs to be changed
        self.AwayfromObstacle = 10# 评价函数中远离障碍物的权重
        self.MaxVelW = 0.001 # 评价函数中速度最大化的权重
        self.RobotStuckLimitV = 0
        self.RobotStuckLimitW = 0 # 判断机器人是否被卡住
        self.VelAddInterval = 0.0001 # no use
        self.WAddInerval = 0.001 # no use
        self.MaxDis = 5000 # 评价函数计算中用来归一化 no use

class DWA(object):
    def __init__(self, MAX_EDGE_LEN=5000, MAX_TRY_COUNT=5000):
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.MAX_TRY_COUNT = MAX_TRY_COUNT # DWA最多尝试次数，超过则认为找不到
        self.minx = 0 # 地图边界
        self.maxx = 1984
        self.miny = 0
        self.maxy = 1984
        self.action = Action()
        #self.debugger = Debugger()
        self.robotPara = Robot()
        self.maxdis = 10

    def floatrange(self,start,stop,steps):
        ans = [start]
        while ans[-1]<stop:
            ans.append(ans[-1]+steps)
        return ans

    def plan(self, start_x, start_y, start_angel, goal_x, goal_y, goal_angle, vision, disRedius):
        #initialize: cur_x, cur_y, cur_angle, cur_vx, cur_vw
        cur = [start_x, start_y, start_angel, 0, 0]
        flag = 0
        path_x = [start_x]
        path_y = [start_y]
        time_start = time.time()
        #print("start_x: "+str(start_x)+" start_y: "+str(start_y))
        self.action.sendCommand(vx=0, vw=0)
        for i in range(1, self.MAX_TRY_COUNT):
            new_v, new_w = self.stepPlan(cur, goal_x, goal_y, goal_angle, vision)
            # prevent the robot to be stuck
            """ if abs(new_v)<self.robotPara.RobotStuckLimitV and abs(new_w)<self.robotPara.RobotStuckLimitW:
                if abs(new_v) <1e-6 and abs(new_w) <1e-6:
                    print("Robot is error.")
                    new_w = self.robotPara.maxW/10
                    new_v = -self.robotPara.maxVel/10
                    self.action.sendCommand(vx=new_v, vw=new_w)
                    rospy.sleep(0.1)
                    self.action.sendCommand(vx=0, vw=0)
                    continue """
            time_end = time.time()
            time_interval = time_end-time_start
            # print("time_interval: "+str(time_interval))
            #if time_interval<self.robotPara.TimeInterval:
                #time.sleep(self.robotPara.TimeInterval-time_interval)
            time_start = time_end
            # send command
            if cur[0]<0 or cur[0]>1984 or cur[1]<0 or cur[1]>1984:
                self.action.sendCommand(vx=0, vw=0)
            else:
                self.action.sendCommand(vx=new_v,  vw=new_w) # 由于差分驱动，所以vy=0
            # print("vx: "+str(new_v)+" vw: "+str(new_w))

            # termination condition
            # x,y不太准，因为前面公式好像就不完全准，所以用vision重新读到的x,y,angle代替
            cur_x, cur_y, cur_angle = vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
            if cur_angle<0:
                cur_angle += 2*math.pi
            distance = math.hypot(cur_x-goal_x, cur_y-goal_y)
            #print("i= "+str(i)+" cur_pos: "+ str(cur_x)+" , "+str(cur_y)+" , "+str(cur_angle)+" distace: "+str(distance))
            if distance<disRedius:
                flag = 1
                self.action.sendCommand(vx=0,  vw=0)
                break
            cur = [cur_x,cur_y,cur_angle,new_v,new_w]
        
        if flag==1:
            print("Nice! We have got the goal.")
        else:
            print("Sadly, we fail to get the goal.")
            
    def stepPlan(self, cur, goal_x, goal_y, goal_angle, vision):
        #print(cur)
        print(cur,goal_x,goal_y)
        MinV, MaxV, MinW, MaxW = self.getVelWSpace(cur)
        minEvaluation = float("inf")
        bestChoose = [cur[3],cur[4]]
        #print("MinV: "+str(MinV)+" MaxV: "+str(MaxV)+" MinW: "+str(MinW)+ " MaxW: "+str(MaxW))
        Interval_v = (MaxV-MinV)/10
        Interval_w = (MaxW-MinW)/10
        for v in self.floatrange(MinV, MaxV, Interval_v):
            for w in self.floatrange(MinW, MaxW, Interval_w):
                if abs(v)<1e-6 and abs(w)>1e-6:
                    continue
                """ if abs(v)<1e-6:
                    if abs(self.action.steer_angle)>3.14/4:
                        continue
                else:
                    if abs(w*self.robotPara.PredictTime+self.action.steer_angle)>3.14/4:
                        continue """
                # get predict trajectory
                trajectory = self.predictTrajectory(cur, v, w)
                
                # get obstacles
                obstacle_x, obstacle_y = self.getCurrentObstacles(cur,vision.data)
                # get cost
                dif_angle, dif_distance,dif_v, dif_w, dif_finalAngle = self.EGoalHeading(trajectory[-1], goal_x, goal_y, goal_angle)
                #print("dif_distance:" + str(dif_distance))
                ToGoalEvaluation = self.robotPara.ToGoalCostDis*dif_distance+self.robotPara.ToGoalCostAngle*dif_angle+\
                                self.robotPara.ToGoalCostV*dif_v+self.robotPara.ToGoalCostW*dif_w+self.robotPara.ToGoalCostFinalAngle*dif_finalAngle
                AwayObstacleEvaluation = self.robotPara.AwayfromObstacle*self.EObstacleDist(trajectory,obstacle_x,obstacle_y)
                MaxVelEvaluation = self.robotPara.MaxVelW*(self.robotPara.maxVel-trajectory[-1][3])/self.robotPara.maxVel
                #print("v: "+str(v)+" w: "+str(w)+" dif_angle: "+str(dif_angle)+" dif_distance: "+str(dif_distance))
                
                
                SumEvaluation = ToGoalEvaluation+AwayObstacleEvaluation+MaxVelEvaluation
                with open('outputt.txt','a') as f:
                    original_stdout = sys.stdout
                    sys.stdout = f
                    print(cur,goal_x,goal_y)
                    print("v: "+str(v)+" w: "+str(w)+" Goal: "+str(ToGoalEvaluation)+" Obs: "+str(AwayObstacleEvaluation)+" Vel: "+str(MaxVelEvaluation)+" Sum: "+str(SumEvaluation))
                    sys.stdout = original_stdout
                if AwayObstacleEvaluation == float('inf'):
                    continue
                if minEvaluation>SumEvaluation:
                    minEvaluation = SumEvaluation
                    bestChoose = [v,w]
        print(cur,goal_x,goal_y)
        with open('outputt.txt','a') as f:
            original_stdout = sys.stdout
            sys.stdout = f
            print("best: "+str(bestChoose[0])+" "+str(bestChoose[1])+" "+str(minEvaluation))
            sys.stdout = original_stdout
        #print(bestChoose[0],bestChoose[1])
        #if bestChoose[0]<0:
        #exit(0)
        # prevent the robot to be stuck
        if minEvaluation == float('inf'):
            exit(0)
            self.action.sendCommand(vx=-self.robotPara.maxVel/2, vw=0)
            rospy.sleep(0.1)
            self.action.sendCommand(vx=0,vw=0)
            bestChoose[0] = 0
            bestChoose[1] = 0
            print("Robot is stuck.")
        #print("v: "+str(bestChoose[0])+" w: "+str(bestChoose[1]))
        return bestChoose[0], bestChoose[1]



    def EGoalHeading(self, trajectory, goal_x, goal_y, goal_angle):
        x = trajectory[0]
        y = trajectory[1]
        angle = trajectory[2]
        v = trajectory[3]
        w = trajectory[4]
        error_angle = math.atan2(y-goal_y, goal_x-x)
        if error_angle<0:
            error_angle += 2*math.pi
        #print(y,goal_y,goal_x,x)
        #print(y-goal_y, goal_x-x)
        #print("error_angle: "+str(error_angle)+" angle: "+str(angle))
        differ_angle = abs(error_angle-angle)
        # differ_distance = math.hypot(x-goal_x, y-goal_y)/math.hypot(self.minx-self.maxx,self.miny-self.maxy)
        differ_distance = math.hypot(x-goal_x, y-goal_y)
        
        # differ_v = (1/differ_distance)*abs(v)
        # differ_w = (1/differ_distance)*abs(w)
        differ_v = abs(v)
        differ_w = abs(w)
        if math.hypot(x-goal_x, y-goal_y)<6:
            differ_finalAngle = abs(angle-goal_angle)
        else:
            differ_finalAngle = 0
        #print("curX: "+str(x)+" curY: "+str(y)+" goal_x: "+str(goal_x)+" goal_y: "+str(goal_y))
        #print("differ_distance:" + str(differ_distance))
        return differ_angle, differ_distance, differ_v, differ_w, differ_finalAngle

    def EObstacleDist(self, trajectory, obstacle_x, obstacle_y):
        minDis = float("inf")
        #print(trajectory[len(trajectory)-1][0],trajectory[len(trajectory)-1][1])
        for i in range(0, len(trajectory)):
            x=trajectory[i][0];  y=trajectory[i][1]
            #print(x,y,obstacle_x,obstacle_y)
            for j in range(0, len(obstacle_x)):
                dis = math.hypot(obstacle_x[j], obstacle_y[j])
                if dis<self.robotPara.robot_size+self.robotPara.avoid_dist:
                    return float("inf")
                if (x<self.minx or x>self.maxx) and (y>self.maxy or y<self.miny):
                    return float('inf')
                minDis = min(minDis, dis)
        #print(minDis)
        #print(1.0/minDis*(math.hypot(self.minx-self.maxx,self.miny-self.maxy)))
        return 1.0/minDis*(math.hypot(self.minx-self.maxx,self.miny-self.maxy))

    def predictTrajectory(self, cur, v, w):
        trajectory = [cur]
        for t in self.floatrange(self.robotPara.TimeInterval, self.robotPara.PredictTime, self.robotPara.TimeInterval):
            cur = self.robotMoveModel(cur, v, w, self.robotPara.TimeInterval)
            if cur[0]<0 or cur[0]>1984 or cur[1]<0 or cur[1]>1984:
                break
            #print("cur[0]: "+str(cur[0])+" cur[1]: "+str(cur[1]))
            trajectory.append(cur)
        #print(trajectory)
        return trajectory
    def getCurrentObstacles(self,cur, AMCLLaserData):
        obstacle_x = []
        obstacle_y = []
        for i in range(0, AMCLLaserData.range_count):
            angle_min = AMCLLaserData.angle_min
            angle_increment = AMCLLaserData.angle_increment
            angle = angle_min+i*angle_increment
            x = AMCLLaserData.ranges[i]*math.cos(angle+cur[2])
            y = -AMCLLaserData.ranges[i]*math.sin(angle+cur[2])
            obstacle_x.append(x)
            obstacle_y.append(y)
            #print(AMCLLaserData.ranges[i])
            #print("x: "+str(x)+" y: "+str(y))
        """ for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y) """
        return obstacle_x, obstacle_y
    # def robotMoveModel(self, cur, v, w, t):
    #     """
    #     Simulate Ackermann steering for a coordinate system where the positive y-direction is downwards.
        
    #     Parameters:
    #     - cur_pos: Current position of the car as a tuple (x, y, theta) where
    #     x and y are the current coordinates of the car, and theta is the current orientation.
    #     - v: The speed of the car in meters per second.
    #     - steering_angle: The steering angle of the front wheels in radians.
    #     - t: The time duration of the movement in seconds.
    #     - L: The length of the car (distance between front and rear axles).
        
    #     Returns:
    #     - A tuple (new_x, new_y, new_theta) representing the new position and orientation of the car.
    #     """
    #     L = 3
    #     if w == 0:  # Straight line movement
    #         new_x = cur[0] + v * math.cos(cur[2]) * t
    #         new_y = cur[1] - v * math.sin(cur[2]) * t  # Invert y-axis movement
    #         new_angle = cur[2]
    #     else:
    #         # Turning radius for the rear axle
    #         R = L / math.tan(w)
            
    #         # Instantaneous Center of Curvature (ICC)
    #         ICC_x = cur[0] - R * math.sin(cur[2])
    #         ICC_y = cur[1] + R * math.cos(cur[2])  # No change needed here, as this is a position calculation
            
    #         delta_theta = v / R * t
    #         new_x = math.cos(delta_theta) * (cur[0] - ICC_x) - math.sin(delta_theta) * (cur[1] - ICC_y) + ICC_x
    #         new_y = math.sin(delta_theta) * (cur[0] - ICC_x) + math.cos(delta_theta) * (cur[1] - ICC_y) + ICC_y
    #         new_y = 2*ICC_y - new_y  # Reflect the y-axis movement across the ICC_y axis
    #         new_angle = cur[2] + delta_theta

        # Normalize the angle
        # new_angle = (new_angle + 2 * math.pi) % (2 * math.pi)
        # print(v,w,new_x,new_y,new_angle,t)
        # return [new_x, new_y, new_angle, v, w]
    def robotMoveModel(self, cur, v, w, t):
        steering_angle = w*t
        new_angle = cur[2] + steering_angle
        if abs(w)<1e-6 :
            new_x = cur[0] + v*t*math.cos(cur[2])
            new_y = cur[1] - v*t*math.sin(cur[2])
        else:
            abs_velocity = abs(v)
            if abs_velocity < 1e-6:
                new_x = cur[0]
                new_y = cur[1]
            else:
                turning_radius = abs_velocity / steering_angle
                theta_change = abs_velocity * t / turning_radius
                new_x = turning_radius * (math.sin(cur[2] + theta_change) - math.sin(cur[2]))/0.05 + cur[0]
                new_y = -turning_radius * (math.cos(cur[2] + theta_change) - math.cos(cur[2]))/0.05 + cur[1]
        return [new_x, new_y, new_angle, v, w]

    def getVelWSpace(self, cur):
        v_left = cur[3] - self.robotPara.TimeInterval*self.robotPara.maxAddVel
        v_right = cur[3] + self.robotPara.TimeInterval*self.robotPara.maxAddVel
        w_left = cur[4] - self.robotPara.TimeInterval*self.robotPara.maxAddW
        w_right = cur[4] + self.robotPara.TimeInterval*self.robotPara.maxAddW
        Vd = [v_left, v_right, w_left, w_right]
        Vs = [-self.robotPara.maxVel, self.robotPara.maxVel, -self.robotPara.maxW, self.robotPara.maxW]
        VSpace = [max(Vd[0], Vs[0]), min(Vd[1], Vs[1]), max(Vd[2], Vs[2]), min(Vd[3],Vs[3])]
        return VSpace[0], VSpace[1], VSpace[2], VSpace[3]




    
