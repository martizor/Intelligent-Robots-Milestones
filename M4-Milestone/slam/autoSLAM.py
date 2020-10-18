# Manually drive the robot inside the arena and perform SLAM using ARUCO markers

# Import packages
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import json
import time

# Import keyboard teleoperation components
import penguinPiC
import keyboardControlARtestStarter as Keyboard

# Import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import Slam as Slam
import Robot as Robot
import aruco_detector as aruco
import Measurements as Measurements

# Manual SLAM
class Operate:
    def __init__(self, datadir, ppi):
        # Initialise
        self.ppi = ppi
        self.ppi.set_velocity(0, 0)
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)

        ##Arrays used to store Aruco markers and routes
        self.taglist = []
        self.checked_markers = []
        self.marker_list = []
        self.path = []
        self.saved_map = []

        # Get camera / wheel calibration info for SLAM
        camera_matrix, dist_coeffs, scale, baseline = self.getCalibParams(datadir)
        self.keyboard = Keyboard.Keyboard(self.ppi)

        # SLAM components
        self.pibot = Robot.Robot(baseline, scale, camera_matrix, dist_coeffs)
        self.aruco_det = aruco.aruco_detector(self.pibot, marker_length=0.1)
        self.slam = Slam.Slam(self.pibot)
        self.stage = 1
        self.count = 1
        self.flag = 1
        self.K_pv = 2.3
        self.K_pw = 4
        self.scale = scale
        self.baseline = baseline
        self.current_marker = "start"


    #def __del__(self):
        #self.ppi.set_velocity(0, 0)

    def getCalibParams(self, datadir):
        # Imports camera / wheel calibration parameters
        fileK = "{}camera_calibration/intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}camera_calibration/distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        fileS = "{}wheel_calibration/scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "{}wheel_calibration/baseline.txt".format(datadir)
        baseline = np.loadtxt(fileB, delimiter=',')

        return camera_matrix, dist_coeffs, scale, baseline

    def control(self,lv,rv):
        # Import teleoperation control signals
        drive_meas = Measurements.DriveMeasurement(lv, rv, dt=0.3)
        self.slam.predict(drive_meas)

    def vision(self):
        # Import camera input and ARUCO marker info
        self.img = self.ppi.get_image()
        lms, aruco_image = self.aruco_det.detect_marker_positions(self.img)
        self.slam.add_landmarks(lms)
        self.slam.update(lms)

    def get_currentMarker(self):
        self.img = self.ppi.get_image()
        current_marker = self.aruco_det.get_current_id(self.img)
        return current_marker

    def get_currentMarker_ID(self):
        self.img = self.ppi.get_image()
        current_marker = self.aruco_det.get_ID(self.img)
        return current_marker

    def move(self,lv,rv):
        self.ppi.set_velocity(lv,rv)


    def get_distance(self,slam):
        index = len(slam.taglist) - 1
        map_list = slam.markers.tolist()
        robotPosition = self.slam.robot.state
        if (index >= 0):
            distance = np.sqrt((map_list[0][index]-robotPosition[0])**2+(map_list[1][index]-robotPosition[1])**2)
            return distance[0]
        else:
            return None

    def clamp_angle(self, rad_angle=0, min_value=-np.pi,max_value=np.pi):
        if min_value > 0:
            min_value *= 1
        angle = (rad_angle + max_value) % (2 * np.pi) + min_value

        return angle

    def get_angle_robot_to_goal(self,robot_state=np.zeros(3),goal=np.zeros(2)):
        x_goal, y_goal, _ = goal
        x,y,theta = robot_state
        x_diff = x_goal - x
        y_diff = y_goal - y
        alpha = self.clamp_angle(np.arctan2(y_diff,x_diff)-theta)

        return alpha

    def get_distance_robot_to_goal(self,robot_state=np.zeros(3),goal=np.zeros(2)):
        x_goal, y_goal, _ = goal
        x,y,theta = robot_state
        x_diff = x_goal - x
        y_diff = y_goal - y
        rho = np.hypot(x_diff,y_diff)
        return rho


    def move_to_goal_controller(self, cur_pos, goal_pos, delta_time = 0.3):
        ## cur_pos --> Current Position Marker [0,0]
        ## Make the robot Stop a bit before it goes:

        distance_to_goal = self.get_distance_robot_to_goal(cur_pos,goal_pos)
        desired_heading = self.get_angle_robot_to_goal(cur_pos,goal_pos)

        while distance_to_goal > 0.45:
            print("Distance: {}".format(distance_to_goal))
            ## Computing the control inputs (Linear Velocity and Angular Velocity)
            v_k = self.K_pv*distance_to_goal
            w_k = self.K_pw*desired_heading

            ## Convert Linear and Angular velocity into left and right wheel speeds:
            right_speed, left_speed = self.revert_wheel_speeds(v_k,w_k)

            ## Limiting the wheel speeds to 40 ticks/s
            if right_speed > left_speed:
                ratio = left_speed/right_speed
                if (np.abs(right_speed) > 50):
                    right_speed = np.clip(right_speed,-50,50,out=None)
                left_speed = right_speed*ratio
            else:
                ratio = right_speed/left_speed
                if np.abs(left_speed) > 50:
                    left_speed = np.clip(left_speed,-50,50,out=None)
                right_speed = left_speed*ratio


            ## Calling the self.move methods to move the penguin pi in gazebo
            self.move(int(left_speed),int(right_speed))


            ## Use self.control to compute the predicted pose of the robot
            self.control(int(left_speed),int(right_speed))

            ## Vision Step
            self.vision()

            self.write_map(self.slam)

            cur_pos = self.slam.robot.state
            new_state = np.array([cur_pos[0][0],cur_pos[1][0],cur_pos[2][0]])

            ## Update the distance and desired heading with the new State:
            distance_to_goal = self.get_distance_robot_to_goal(new_state,goal_pos)
            desired_heading = self.get_angle_robot_to_goal(new_state,goal_pos)

        self.move(0,0)
        print("PD FINISHED")





    def revert_wheel_speeds(self, linear_velocity, angular_velocity):
        right_speed = linear_velocity + (angular_velocity*self.pibot.wheels_width)/2
        left_speed = linear_velocity - (angular_velocity*self.pibot.wheels_width)/2

        return right_speed/self.pibot.wheels_scale , left_speed/self.pibot.wheels_scale

    def write_map(self, slam):
        # Output SLAM map as a json file
        map_dict = {"AR_tag_list":slam.taglist,
                    "map":slam.markers.tolist(),
                    "covariance":slam.P[3:,3:].tolist()}
        with open("slam.txt", 'w') as map_f:
            json.dump(map_dict, map_f, indent=2)

    def process(self):
        fps = 15
        flag = 0
        measurements = []
        seen_ids = []
        stop =  8
        while len(self.checked_markers) < stop:
            ## The first stage:
            # Robot robot rotates 360 degrees at a speed of 45 tick/s to find nearby arco markers.
            # The x and y coordinates of the aruco markers as well the distance to the aruco marker from the robot is stored
            if (self.stage == 1):
                if (flag == 0):
                    for step in range(int(4.3*fps)):
                        # spinning and looking for markers at each step
                        self.ppi.set_velocity(-45, 45, 1/fps)
                        ids = self.get_currentMarker()
                        self.control(-45,45)
                        self.vision()
                        if ids is None:
                            continue
                        else:
                            for i in range(len(ids)):
                                idi = ids[i,0]
                                if idi in seen_ids:
                                    continue
                                else:
                                    seen_ids.append(idi)
                                ## Accessing the robot position and the predicted position of the aruco marker:
                                ## Using the robot position and predicted position of the aruco marker --> Distance is computed
                                robotPosition = self.slam.robot.state
                                index = len(self.slam.taglist) - 1
                                map_list = self.slam.markers.tolist()
                                dist = np.sqrt((map_list[0][index]-robotPosition[0]) ** 2 + (map_list[1][index]-robotPosition[1]) ** 2)
                                # save marker measurements and distance
                                lm_measurement = [idi, dist, map_list[0][index], map_list[1][index]]
                                measurements.append(lm_measurement)
                                # ------------------------------------------------------------------------------------
                    print("Finish Spinning:")
                    start = time.process_time()
                    ## After the robot has finished rotating the robot will then turn to the nearest Aruco Marker!
                    flag = 1
                    self.move(0,0)
                    time.sleep(1.8)
                    ## Remove Already Checked off Aruco markers:
                    filtered_measurements = []
                    for i in range(len(measurements)):
                        if (measurements[i][0] not in self.checked_markers):
                            print("Entered Append Statement!")
                            filtered_measurements.append(measurements[i])
                        else:
                            continue

                    measurements = filtered_measurements
                    measurements = sorted(measurements, key=lambda x: x[1]) # sort seen markers by distance (closest first)
                    ## If Aruco 8 is detected before Aruco 21: Remove Aruco 21 from the measurment array
                    if (8 in self.checked_markers and 21 not in self.checked_markers):
                        print("Aruco 8 was detected before Aruco 21!!!")
                        print(measurements)
                        for i in range(len(measurements)):
                            if (measurements[i][0] == 21):
                                measurements.pop(i)
                                break
                        ## Append Aruco 21 into the self_checked_marker so if statement never runs again!
                        self.checked_markers.append(21)


                    if len(measurements) > 0:
                        # add discovered markers to map
                        for accessible_marker in measurements:
                            if self.current_marker != accessible_marker[0]: # avoid adding path to self
                                self.path = []
                                self.path.append(self.current_marker)
                                self.path.append(accessible_marker[0])
                                self.path.append(accessible_marker[1])
                                self.saved_map.append(self.path)
                                if accessible_marker[0] not in [found[0] for found in self.marker_list]: # avoid adding repeated marker
                                    self.marker_list.append([accessible_marker[0], accessible_marker[2], accessible_marker[3]])
                            else:
                                continue
                    else:
                        ##IF NO NEW ARUCO MARKERS ARE DETECTED THEN BREAK THE PROGRAM!
                        self.move(0,0)
                        stop = len(self.checked_markers)
                        break

                self.move(-35,35)
                self.control(-35,35)
                self.vision()
                current_markerID = self.get_currentMarker_ID()
                print(measurements)
                print("Robot Sees: {}".format(current_markerID))
                print("Closest Aruco: {}".format(measurements[0][0]))
                ## Conditional Statement to check if the robot is at the nearest marker!
                if (current_markerID == measurements[0][0]):
                    if (current_markerID not in self.checked_markers):
                        self.move(0,0)
                        self.control(0,0)
                        self.vision()
                        print("STOP AT ARUCO MARKER")
                        self.checked_markers.append(current_markerID)
                        self.stage = 2

                print(time.process_time() - start)
                if (time.process_time() - start > 16):
                    self.move(0,0)
                    stop = len(self.checked_markers)
                    break

            ## Drive to the aruco marker
            elif (self.stage == 2):
                self.move(0,0)
                self.control(0,0)
                self.vision()
                ## Stop The Robot for a BIT!
                time.sleep(1.5)
                robotPosition = self.slam.robot.state
                index = len(self.slam.taglist) - 1
                map_list = self.slam.markers.tolist()
                ##Average Goal Position:

                avg_goalx = (measurements[0][2] + map_list[0][index])/2
                avg_goaly = (measurements[0][3] + map_list[1][index])/2

                if (len(self.checked_markers) == 1):
                    dist_slam = np.sqrt((map_list[0][index]-0) ** 2 + (map_list[1][index]-0) ** 2)
                    dist_measurment = np.sqrt((measurements[0][2]-0) ** 2 + (measurements[0][3]-0) ** 2)
                    dist_avg = np.sqrt((avg_goalx-0) ** 2 + (avg_goaly -0) ** 2)
                    print("Distance SLAM: {}".format(dist_slam))
                    print("Distance MEASURMENT: {}".format(dist_measurment))
                    print("Distance AVG: {}".format(dist_avg))
                    if (dist_slam > dist_measurment and dist_slam < dist_avg):
                        print("Dist SLAM is the MIDDLE")
                        robotPosition[0][0] = 0
                        robotPosition[1][0] = 0
                        goal_x = map_list[0][index]
                        goal_y = map_list[1][index]
                    elif (dist_measurment > dist_slam and dist_measurment < dist_avg):
                        print("Dist measurement is the middle")
                        robotPosition[0][0] = 0
                        robotPosition[1][0] = 0
                        goal_x = measurements[0][2]
                        goal_y = measurements[0][3]
                    else:
                        print("Dist AVG is MIDDLE")
                        robotPosition[0][0] = 0
                        robotPosition[1][0] = 0
                        goal_x = avg_goalx
                        goal_y = avg_goaly
                else:
                        dist_slam = np.sqrt((map_list[0][index]-goal_x) ** 2 + (map_list[1][index]-goal_y) ** 2)
                        dist_measurment = np.sqrt((measurements[0][2]-goal_x) ** 2 + (measurements[0][3]-goal_y) ** 2)
                        dist_avg = np.sqrt((avg_goalx-goal_x) ** 2 + (avg_goaly -goal_y) ** 2)
                        print("Distance SLAM: {}".format(dist_slam))
                        print("Distance MEASURMENT: {}".format(dist_measurment))
                        print("Distance AVG: {}".format(dist_avg))

                        if (dist_slam > dist_measurment and dist_slam > dist_avg):
                            print("Dist SLAM is the MAX")
                            robotPosition[0][0] = goal_x
                            robotPosition[1][0] = goal_y
                            goal_x = map_list[0][index]
                            goal_y = map_list[1][index]
                        elif (dist_measurment > dist_slam and dist_measurment > dist_avg):
                            print("Dist measurement is the MAX")
                            robotPosition[0][0] = goal_x
                            robotPosition[1][0] = goal_y
                            goal_x = measurements[0][2]
                            goal_y = measurements[0][3]
                        else:
                            print("Dist AVG is MAX")
                            robotPosition[0][0] = goal_x
                            robotPosition[1][0] = goal_y
                            goal_x = avg_goalx
                            goal_y = avg_goaly



                state = np.array([robotPosition[0][0],robotPosition[1][0],robotPosition[2][0]])
                goal_pos = np.array([goal_x,goal_y,0])
                ## Entering PD Function
                self.move_to_goal_controller(state,goal_pos, delta_time=0.3)

                self.move(0,0)
                self.control(0,0)
                self.vision()
                robotPosition = self.slam.robot.state
                dist_slam = np.sqrt((map_list[0][index]-robotPosition[0]) ** 2 + (map_list[1][index]-robotPosition[1]) ** 2)
                dist_measurment = np.sqrt((measurements[0][2]-robotPosition[0]) ** 2 + (measurements[0][3]-robotPosition[1]) ** 2)
                dist_avg = np.sqrt((avg_goalx-robotPosition[0]) ** 2 + (avg_goaly -robotPosition[1]) ** 2)
                print("Distance AFTER SLAM: {}".format(dist_slam))
                print("Distance AFTER MEASURMENT: {}".format(dist_measurment))
                print("Distance AFTER AVG: {}".format(dist_avg))

                ##Set the new current_position of the robot:
                self.current_marker = current_markerID
                ## Loop back to stage (Rotate until an aruco marker is found)
                flag = 0
                self.move(0,0)
                time.sleep(1)
                ##Writing Markers and Route to Map.txt
                self.marker_list = sorted(self.marker_list, key=lambda x: x[0])
                with open('map.txt','w') as f:
                    f.write('id, x, y\n')
                    for markers in self.marker_list:
                        for marker in markers:
                            f.write(str(marker) + ',')
                        f.write('\n')
                    f.write('\ncurrent id, accessible id, distance\n')
                    for routes in self.saved_map:
                        for route in routes:
                            f.write(str(route) + ',')
                        f.write('\n')
                print('map saved!')
                self.stage = 1

        print("CODE HAS TERMINATED!!")
        print("Marker list:")
        print(self.marker_list)
        print("Saved Map:")
        print(self.saved_map)






if __name__ == "__main__":
    # Location of the calibration files
    currentDir = os.getcwd()
    datadir = "{}/calibration/".format(currentDir)
    # connect to the robot
    ppi = penguinPiC.PenguinPi()

    # Perform Manual SLAM
    operate = Operate(datadir, ppi)
    operate.process()
