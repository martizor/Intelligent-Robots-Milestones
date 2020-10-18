# teleoperate the robot through keyboard control
# getting-started code

from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np


#import OpenCv ARUCO functions
import cv2.aruco as aruco

class Keyboard:

    def __init__(self, ppi=None):
        # storage for key presses
        self.directions = [False for _ in range(4)]
        self.signal_stop = False

        # connection to PenguinPi robot
        self.ppi = ppi
        self.wheel_vels = [0, 0]
        self.speed  = 1

        self.listener = Listener(on_press=self.on_press, on_release = self.on_release).start()


    ## Method which used to check if a key is being released:
    def on_release(self,key):
        self.directions = [False,False,False,False]
        self.send_drive_signal()
    ## Method which is used to check if a key is being pressed:
    def on_press(self, key):

        # use arrow keys to drive, space key to stop
        # feel free to add more keys
        if key == Key.up:
            self.directions = [True,False,False,False]
        elif key == Key.down:
            self.directions = [False,True,False,False]
        elif key == Key.left:
            self.directions = [False,False,True,False]
        elif key == Key.right:
            self.directions = [False,False,False,True]
        elif key == Key.space:
            self.signal_stop = True
            self.directions = [False,False,False,False]

        self.send_drive_signal()



    def get_drive_signal(self):
        # translate the key presses into drive signals
        ## Going Forward:
        if (self.directions[0] == True):
            left_speed = 50*self.speed
            right_speed = 50*self.speed
        ## Going Backwards:
        elif (self.directions[1] == True):
            left_speed = -50*self.speed
            right_speed = -50*self.speed
        ## Stopping:
        elif (self.signal_stop == True):
            left_speed = 0
            right_speed = 0
            self.signal_stop = False
        # Turning Left:
        elif (self.directions[2] == True):
            left_speed = -50*self.speed
            right_speed = 50*self.speed
        ## Turning Right:
        elif (self.directions[3] == True):
            left_speed = 50*self.speed
            right_speed = -50*self.speed
        else:
            left_speed = 0
            right_speed = 0

        return left_speed, right_speed

    def send_drive_signal(self):
        if not self.ppi is None:
            lv, rv = self.get_drive_signal()
            lv, rv = self.ppi.set_velocity(lv, rv)
            self.wheel_vels = [lv, rv]


    def latest_drive_signal(self):
            return self.wheel_vels

    def latest_speed(self):
        return self.speed






if __name__ == "__main__":
    import penguinPiC
    ppi = penguinPiC.PenguinPi()

    keyboard_control = Keyboard(ppi)

    cv2.namedWindow('video', cv2.WINDOW_NORMAL);
    cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);

    while True:
        # font display options
        font = cv2.FONT_HERSHEY_SIMPLEX
        location = (0, 0)
        font_scale = 1
        font_col = (255, 255, 255)
        line_type = 2
        font_col2 = (255,0,0)

        # get velocity of each wheel
        wheel_vels = keyboard_control.latest_drive_signal();
        Speed_val = keyboard_control.latest_speed();
        L_Wvel = wheel_vels[0]
        R_Wvel = wheel_vels[1]


        # get current camera frame
        curr = ppi.get_image()

        #uncomment to see how noises influence the accuracy of ARUCO marker detection
        #im = np.zeros(np.shape(curr), np.uint8)
        #cv2.randn(im,(0),(99))
        #curr = curr + im

        # show ARUCO marker detection annotations (Adding noise into the system)
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.minDistanceToBorder = 0
        aruco_params.adaptiveThreshWinSizeMax = 1000
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)

        aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares


        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

        # feel free to add more GUI texts

        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)
        ## Speed Changer:
        cv2.putText(resized, 'PenguinPi Speed: x' + str(Speed_val), (15, 200), font, font_scale, font_col, line_type)

        ## Notifiying the direction that the user is moving:
        if (L_Wvel > 0 and R_Wvel > 0):
            cv2.putText(resized, 'Forward', (15, 100), font, font_scale, font_col2, line_type)
        elif (L_Wvel < 0 and R_Wvel < 0):
            cv2.putText(resized, 'Reverse', (15, 100), font, font_scale, font_col2, line_type)
        elif (L_Wvel <= -30 and R_Wvel >= 30):
            cv2.putText(resized, 'Left', (15, 100), font, font_scale, font_col2, line_type)
        elif (L_Wvel >= 30 and R_Wvel <= -30):
            cv2.putText(resized, 'Right', (15, 100), font, font_scale, font_col2, line_type)
        else:
            cv2.putText(resized, 'Stationary', (15, 100), font, font_scale, font_col2, line_type)




        cv2.imshow('video', resized)
        cv2.waitKey(1)

        continue
