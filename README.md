# Intelligent-Robots-Milestones:
Watch the youtube video below: 
Tutorial on how to navigate through our files and a video of our robot navigating around the map
https://www.youtube.com/watch?v=jX_OU6e4xmM&ab_channel=martinyang

**Important Things to Note** 
BEST MAP: 
Gazebo Frame Rate: 50.5 
Time taken: 5 min and 10 Seconds 

- The pose of our robot is estimating using SLAM. This estimated pose is then fed into PD controller which manipulates the left and right wheel velocities of the robot to accurately drive towards the closest aruco marker. 
- We forgot to mention this in the video but there are times that the robot could get stuck. This could happen every now and then. But usually the robot can confidently go around the map. If the robot does stuck, if you have the time we would want you to ideally execute our autoSLAM.py script once more. 
- If you decide to run our autoSLAM.py script more than once, be aware that an INDEX error might pop up. If that ever happens just execute python script again and the error should go away. 
