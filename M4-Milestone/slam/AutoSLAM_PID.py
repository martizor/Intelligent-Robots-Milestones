# Have the robot drive itself around the arena, using PID to locate itself 

# Import packages
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import json

# Import keyboard teleoperation components
import penguinPiC
import keyboardControlARtestStarter as Keyboard

# Import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import slam.Slam as Slam
import slam.Robot as Robot
import slam.aruco_detector as aruco
import slam.Measurements as Measurements

# Manual SLAM
class Operate:
    def __init__(self, datadir, ppi):
        # Initialise
        self.ppi = ppi
        self.ppi.set_velocity(0, 0)
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)

        # Keyboard teleoperation components
        self.keyboard = Keyboard.Keyboard(self.ppi)

        # Get camera / wheel calibration info for SLAM
        camera_matrix, dist_coeffs, scale, baseline = self.getCalibParams(datadir)

        # SLAM components
        self.pibot = Robot.Robot(baseline, scale, camera_matrix, dist_coeffs)
        self.aruco_det = aruco.aruco_detector(self.pibot, marker_length=0.1)
        self.slam = Slam.Slam(self.pibot)

        # PID components
        self.K_rho = 3
        self.K_alpha = 7
        self.K_beta = -1
        self.PID_threshold = 0.8

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

    def control(self):
        # Import teleoperation control signals
        lv, rv = self.keyboard.latest_drive_signal()
        drive_meas = Measurements.DriveMeasurement(lv, rv, dt=0.3)
        self.slam.predict(drive_meas)

    def vision(self):
        # Import camera input and ARUCO marker info
        self.img = self.ppi.get_image()
        lms, aruco_image = self.aruco_det.detect_marker_positions(self.img)
        self.slam.add_landmarks(lms)
        self.slam.update(lms)

    def display(self, fig, ax):
        # Visualize SLAM
        ax[0].cla()
        self.slam.draw_slam_state(ax[0])

        ax[1].cla()
        ax[1].imshow(self.img[:, :, -1::-1])

        plt.pause(0.01)

    def write_map(self, slam):
        # Output SLAM map as a json file
        map_dict = {"AR_tag_list":slam.taglist,
                    "map":slam.markers.tolist(),
                    "covariance":slam.P[3:,3:].tolist()}
        with open("slam.txt", 'w') as map_f:
            json.dump(map_dict, map_f, indent=2)

    def process(self):
        # Show SLAM and camera feed side by side
        fig, ax = plt.subplots(1, 2)
        img_artist = ax[1].imshow(self.img)
        self.display(fig, ax)
        # self.ppi.set_velocity(-20, 20)

        print("Using PID")
        self.move_to_goal_controller([0,0,0], [3.11284, 0.882415, 0], delta_time=0.3)
        # self.move_to_goal_controller([1.665, 0.1835, 2.0], [-1.075032, 4.12235, 2.0 ], delta_time=0.3)
        print("exited PID loop")
        # Main loop
        while True:
            # Run the PID controller (i think this should be put in place of the current controller self.control())
            # Run SLAM
            # self.control()
            # PID controller
            
            # self.vision()

            # Save SLAM map
            self.write_map(self.slam)

            # Output visualisation
            self.display(fig, ax)
            exit()

    def move_to_goal_controller(self, cur_pos, goal_pos, delta_time = 0.3):
        fig, ax = plt.subplots(1, 2)
        img_artist = ax[1].imshow(self.img)
        self.pibot.set_state(cur_pos[0], cur_pos[1], cur_pos[2]) # so that SLAM works
        
        # Terminology used is the same as that used in the lecture notes

        # First rho, alpha and beta
        alpha = np.arctan2((goal_pos[1] - cur_pos[1]),(goal_pos[0] - cur_pos[0])) - cur_pos[2]
        rho = np.sqrt((goal_pos[0] - cur_pos[0])**2 + (goal_pos[1] - cur_pos[1])**2)
        beta = goal_pos[2] - cur_pos[2] - alpha
        beta = np.clip(beta, -np.pi, np.pi)
        #leaving beta out for now
        # is the orientation of the ARUCO marker known?
        print("Driving PID")

        # self.ppi.set_velocity(-50,50)

        # Perform a single EKF step:
        while rho > self.PID_threshold:
            # 1: Compute new control input
            v_k = self.K_rho * rho
            w_k = self.K_alpha*alpha + self.K_beta*beta

            v_k = np.clip(v_k, -1, 1, out=None)
            w_k = np.clip(w_k, -1, 1, out=None)

             #print("Speed: %f, Angular velocity: %f" %(v_k, w_k))

            # 2. Apply control to the robot and get a new state
            # convert v_k and w_k into wheel speeds for the robot
            right_speed, left_speed = self.revert_wheel_speeds(v_k, w_k)

            # print("Left RAW: %f, Right RAW: %f" %(left_speed, right_speed))

            # This block of code restricts the max speed of the wheels to 30
            # And keeps the ratio of right to left speed in check
            if right_speed > left_speed:
                ratio = left_speed/right_speed
                if np.abs(right_speed) > 30:
                    right_speed = np.clip(right_speed, -30, 30, out=None)
                left_speed = right_speed*ratio
            else:
                ratio = right_speed/left_speed
                if np.abs(left_speed) > 30:
                    left_speed = np.clip(left_speed, -30, 30, out=None)
                right_speed = left_speed*ratio

            # print("Left: %f, Right: %f" %(left_speed, right_speed))

            self.ppi.set_velocity(int(left_speed), int(right_speed))

            # Left and right wheel speeds are received straight from the PPI itself (rather than the keyboard)
            drive_meas = Measurements.DriveMeasurement(left_speed, right_speed, dt=0.3)
            self.slam.predict(drive_meas)

            # Vision step
            self.img = self.ppi.get_image()
            lms, aruco_image = self.aruco_det.detect_marker_positions(self.img)
            self.slam.add_landmarks(lms)
            self.slam.update(lms)

            # Updating the SLAM map
            self.write_map(self.slam)

            # Updating the robot's position (for PID)
            cur_pos = self.pibot.get_state()
            print("X: %f, Y: %f, Theta: %f" %(cur_pos[0], cur_pos[1], cur_pos[2]))

            # 3. Update rho, alpha and beta
            rho = np.sqrt((goal_pos[0] - cur_pos[0])**2 + (goal_pos[1] - cur_pos[1])**2)
            alpha = np.arctan2((goal_pos[1] - cur_pos[1]),(goal_pos[0] - cur_pos[0])) - cur_pos[2]
            beta = goal_pos[2] - cur_pos[2] - alpha
            beta = np.clip(beta, -np.pi, np.pi, out=None)
            # print("Distance to ARUCO marker: %f" %(rho))

            self.display(fig, ax)

            # rho, alpha and beta are then converted into angular and linear velocities, and the PPI
        self.ppi.set_velocity(0,0)
        print("PID finished.")
        

    def revert_wheel_speeds(self, linear_velocity, angular_velocity):
        right_speed = linear_velocity + (angular_velocity*self.pibot.wheels_width)/2
        left_speed = linear_velocity - (angular_velocity*self.pibot.wheels_width)/2

        return right_speed/self.pibot.wheels_scale , left_speed/self.pibot.wheels_scale
            




if __name__ == "__main__":
    # Location of the calibration files
    currentDir = os.getcwd()
    # datadir = "{}/calibration/".format(currentDir)
    datadir = "{}/".format(currentDir)

    # connect to the robot
    ppi = penguinPiC.PenguinPi()

    # Perform Manual SLAM
    operate = Operate(datadir, ppi)
    operate.process()


def convert_wheel_speeds(self, left_speed, right_speed):
    # Convert to m/s
    left_speed_m = left_speed * self.wheels_scale
    right_speed_m = right_speed * self.wheels_scale

    # Compute the linear and angular velocity
    linear_velocity = (left_speed_m + right_speed_m) / 2.0
    angular_velocity = (right_speed_m - left_speed_m) / self.wheels_width
    
    return linear_velocity, angular_velocity



