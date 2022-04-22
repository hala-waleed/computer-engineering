import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, RandomPath, VehicleIcon, RangeBearingSensor, LandmarkMap
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