# Abstract
   This project assessment aims to create a code for a robot using the robotics toolbox to be creative and add our codes so it starts moving. The code was written in software called Linux then we used a visual program called visual studio after this step we used another program called python program which is crucial for the project. We wanted to create a robot which would not be stopped or stumbled. We have determined, thought, and reviewed the codes needed to be implemented in the python program. The code in the python program will allow the robot to move in certain directions and determine obstacles to avoid. After writing the code and implementing it into the program or software we will showcase and review the robot and its programming. This review includes the robot reaching a certain target with obstacles scattered around, the review’s purpose is to see if the robot will avoid, dodge, or crash into the obstacles. This will help determine if the code was falsely executed. This will give us the advantage to understand the downfalls or mistakes of the robot. These mistakes would include adjusting the sensor or rechecking lines of code in the program.

 
 # Introduction 
     Before I apply the code I had to download specific programs so it can work like Ubuntu 18.04, python 3.8, robotics toolbox and it have to be the old software it should not be updated.
The first thing before we program anything we need to put it in a coded form. So we import the variables to help the robot to move. Then we put the inputs for each variable like what we did in our code. Those are the inputs we used to make the robot move.
x = int (input ("please enter initial x coordinate "))
y = int (input ("please enter initial y coordinate "))
A = int (input ("please enter initial Angle "))
N = int (input ("please enter the number of obstacles"))
Tx= int (input ("please enter x coordinate of the target "))
Ty= int (input ("please enter y coordinate of the target ")
The symbols beside each input resemble the input itself. Then we put the x and y axes so it’s plotted on the graph. After it, we put a certain angle that is 45 degrees for robot rotation and direction if there was an obstacle. Moreover, we wanted the robot to move around or between the obstacles without striking them and reach the target. Furthermore, we put a target for the robot on the x and y-axis to go. Then we start to put the values to get the output for the obstacle distance, sensor, obstacles for angels and if we are at risk. 
 
Figure 1initail sensor readings ![](images/fg1.peg)

           
Figure 2: Readings during navigation with no RISK  ![](images/fg2.peg)         Figure 3: Readings during navigation while there is a RISK  ![](images/fg3.peg)
 
 # Results 

This is the video while the code is working, and the graph appears while the robot moves away from the obstacle. It shows the sensor readings and obstacles in the angle and distance. Also, it said the areas that are at risk

 ![](images/results1.peg)

  
          






























                                                                                                     

Areas for innovation of autonomous navigation

The navigation through the obstacles 
def CheckObstacles(SensorReadings, Robot):
        Risk = False
        Obstacles = SensorReadings.h(Robot.x)
        ObstaclesR = Obstacles[:, 0]
        ObstaclesTheta = (Obstacles[:, 1] / pi) * 180
        print('Obstacles Distances: ', ObstaclesR)
        print('Obstacles Angles: ', ObstaclesTheta)
        for i in range(len(ObstaclesR)):
                if ObstaclesR[i] < 10:
                        if abs(ObstaclesTheta[i]) < 45:
                                print('We are at RISK!!!')
                                Risk = True
        return [Risk, ObstaclesR[i], ObstaclesTheta[i]]

flag=True 
while (flag):
        Dy = Target[1] - Robot.x[1] 
        Dx = Target[0] - Robot.x[0]
        TargetAngle = atan2(Dy, Dx)
        SteeringAngle = TargetAngle - Robot.x[2] 
        if abs(Dy)>0.1  and abs(Dx)>0.1:
                flag = True
        else: 
                flag = False
        [Risk, ObsR, ObsA] = CheckObstacles(sensor, Robot)
        if Risk:
                if ObsA >= 0:
                        SteeringAngle = SteeringAngle - (pi/4)
                elif ObsA < 0:
                        SteeringAngle = SteeringAngle + (pi/4)
        Robot.step(3,SteeringAngle)
        Robot._animation.update(Robot.x)
        plt.pause(0.005)
plt.pause(10)










 # Flow chart 
![](images/flow.peg)

 





# Conclusion     
I imported matplotlib. pyplot as plt to make x and y axes to be plotted on the graph. Then I imported from the robotics toolbox Bicycle, Random Path, Vehicle Icon RangeBearingSensor was used for the sensor readings, and Landmark Map so the robot moves on and we used math import pi, atan2 to calculate the distance, angle, and the sensor readings by the program. Then we used def to check out the obstacles of the sensor readings, distance, and angle but if the obstacles are greater than ten we are at risk also if the angle is greater than 45 we are at risk. Furthermore, if it is less than 10 and the angle is 45 then the risk is false. Then we print all. If it said we are at risk it returns. Then we start to put input for the x and y-axis, angle, x and y target and several obstacles. Then we uploaded the robot photo was saved it on the desktop. Then we started to put the robot into the program so it moves on the graph. Then we started to write the code so the sensor work with a robot on the landmark map works on the graph. Also to plot the axis on the graph and sensor readings to be written in the terminal. Moreover, we started to design the target with a diamond shape and yellow colour. Then we started to plot the target and then we paused it for one 1second. Then we calculate the distance, angle, and sensor reading with their obstacle directions. Then we used the if the condition, so the robot and sensor see obstacles while the robot moving. We put the angle in positive and negative. Then we put pausing time before the target and the robot moves. Also, we made time when the graph appears it stays for a certain amount of time.

















# Appendix 
import matplotlib.pyplot as plt
from robotics toolbox import Bicycle, RandomPath, VehicleIcon, RangeBearingSensor, LandmarkMap
from math import pi, atan2

def CheckObstacles(SensorReadings, Robot):
        Risk = False
        Obstacles = SensorReadings.h(Robot.x)
        ObstaclesR = Obstacles[:, 0]
        ObstaclesTheta = (Obstacles[:, 1] / pi) * 180
        print('Obstacles Distances: ', ObstaclesR)
        print('Obstacles Angles: ', ObstaclesTheta)
        for i in range(len(ObstaclesR)):
                if ObstaclesR[i] < 10:
                        if abs(ObstaclesTheta[i]) < 45:
                                print('We are at RISK!!!')
                                Risk = True
        return [Risk, ObstaclesR[i], ObstaclesTheta[i]]

x = int(input("please enter initial x coordinate ")) 
y = int(input("please enter initial y coordinate "))
A = int(input("please enter initial Angle "))
N = int(input ("please enter the number of obstacles"))
Tx= int(input("please enter x coordinate of the target ")) 
Ty= int(input("please enter y coordinate of the target "))
RobotShape = VehicleIcon ("/home/hala/Desktop/practice21/robot.png", scale = 3)
Robot = Bicycle(
        animation = RobotShape,
        control= RandomPath,
        dim = 10,
        x0 = (x, y, A),  
)
Robot.init(plot = True)
Robot._animation.update(Robot.x)
map = LandmarkMap(N, 50)  
map.plot()
plt.pause(1)
sensor = RangeBearingSensor(robot = Robot, map = map, animate = True)
print('Sensor readings: \n', sensor.h(Robot.x))
Target = [Tx,Ty] 
TargetMarker = {
        "marker" : "D",
        "markersize" : 10,
        "color" : "y", 
}
plt.plot(Target[0],Target[1],**TargetMarker)
plt.pause(1)

flag=True 
while (flag):
        Dy = Target[1] - Robot.x[1] 
        Dx = Target[0] - Robot.x[0]
        TargetAngle = atan2(Dy, Dx)
        SteeringAngle = TargetAngle - Robot.x[2] 
        if abs(Dy)>0.1  and abs(Dx)>0.1:
                flag = True
        else: 
                flag = False
        [Risk, ObsR, ObsA] = CheckObstacles(sensor, Robot)
        if Risk:
                if ObsA >= 0:
                        SteeringAngle = SteeringAngle - (pi/4)
                elif ObsA < 0:
                        SteeringAngle = SteeringAngle + (pi/4)
        Robot.step(3,SteeringAngle)
        Robot._animation.update(Robot.x)
        plt.pause(0.005)
plt.pause(10)
