from __future__ import division
from math import pi, sin, cos
from diff_drive.encoder import Encoder
from diff_drive.pose import Pose

class Odometry:
    """Keeps track of the current position and velocity of a
    robot using differential drive.
    """

    def __init__(self):
        self.leftEncoder = Encoder()
        self.rightEncoder = Encoder()
        self.pose = Pose()
        self.lastTime = 0
        self.last_imu_yaw = 0 
        self.curr_imu_yaw = 0
        self.first = True

        self.motor_theta = 0

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks
        
    def setEncoderRange(self, low, high):
        self.leftEncoder.setRange(low, high)
        self.rightEncoder.setRange(low, high)

    def setTime(self, newTime):
        self.lastTime = newTime
        
    def updateLeftWheel(self, newCount):
        self.leftEncoder.update(newCount)

    def updateRightWheel(self, newCount):
        self.rightEncoder.update(newCount)

    def update_imu(self, new_state):
        self.curr_imu_yaw = new_state
        if self.first:
            self.last_imu_yaw = self.curr_imu_yaw
            self.first = False
            
    def sensor_fusion(self,imu_theta,motor_theta, imu_weight):
        if abs(imu_theta-motor_theta)>5:#one is close to 0deg and the other one is close 360deg
            if imu_theta>motor_theta:
                motor_theta+=(2*pi)
            else:
                imu_theta+=(2*pi)
        angle= imu_weight*(imu_theta)+((1-imu_weight)*motor_theta)
        return angle % (2*pi)

    def updatePose(self, newTime):
        """Updates the pose based on the accumulated encoder ticks
        of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        for details.
        """
        leftTravel = self.leftEncoder.getDelta() / self.ticksPerMeter
        rightTravel = self.rightEncoder.getDelta() / self.ticksPerMeter
        deltaTime = newTime - self.lastTime

        deltaTravel = (rightTravel + leftTravel) / 2
        
        # deltaIMU = self.curr_imu_yaw - self.last_imu_yaw

        deltaTheta = (rightTravel - leftTravel) / self.wheelSeparation


        # deltaTheta = (deltaTheta + deltaIMU)/2


        if rightTravel == leftTravel:
            deltaX = leftTravel*cos(self.pose.theta)
            deltaY = leftTravel*sin(self.pose.theta)
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose.x - radius*sin(self.pose.theta)
            iccY = self.pose.y + radius*cos(self.pose.theta)

            deltaX = cos(deltaTheta)*(self.pose.x - iccX) \
                - sin(deltaTheta)*(self.pose.y - iccY) \
                + iccX - self.pose.x

            deltaY = sin(deltaTheta)*(self.pose.x - iccX) \
                + cos(deltaTheta)*(self.pose.y - iccY) \
                + iccY - self.pose.y


        self.motor_theta = (self.motor_theta + deltaTheta) % (2*pi)

        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = self.sensor_fusion(self.curr_imu_yaw,self.motor_theta,0.7)
        self.pose.xVel = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.yVel = 0
        self.pose.thetaVel = deltaTheta / deltaTime if deltaTime > 0 else 0.

        # self.debug = (self.pose.theta + self.curr_imu_yaw)/2
        print(str(self.motor_theta) +","+str(self.curr_imu_yaw) + "," + str(self.pose.theta))
        # self.pose.theta = self.debug
        
        self.lastTime = newTime
        self.last_imu_yaw = self.curr_imu_yaw


    def getPose(self):
        return self.pose

    def setPose(self, newPose):
        self.pose = newPose
