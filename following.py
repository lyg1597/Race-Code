import numpy as np
import time

import rospy

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from popgri_msgs.msg import LaneList
from popgri_msgs.msg import LocationInfo
from ackermann_msgs.msg import AckermannDrive

class ego_waypoint_follower():
    def __init__(self):
        self.waypoint = None
        self.curr_state = None
        self.lane_waypoints = None
        self.clock = None

        self.waypointSub = rospy.Subscriber("/carla/hero0/waypoints", Vector3, self.__waypointHandler, queue_size = 1)
        self.stateSub = rospy.Subscriber("/carla/hero0/location", LocationInfo, self.__stateHandler, queue_size = 1)
        self.lanewaypointSub = rospy.Subscriber("/carla/hero0/lane_waypoints", LaneList, self.__laneWaypointHandler, queue_size = 1)
        self.clockSub = rospy.Subscriber("/clock", Clock, self.__clockHandler, queue_size = 1)

        self.controlPub = rospy.Publisher("/carla/hero0/ackermann_control", AckermannDrive, queue_size = 1)

    def __waypointHandler(self, data):
        self.waypoint = data

    def __stateHandler(self, data):
        self.curr_state = data
    
    def __laneWaypointHandler(self, data):
        self.lane_waypoints = data

    def __clockHandler(self, data):
        self.clock = data

        if self.curr_state == None or self.lane_waypoints == None:
            return
        target_x = self.lane_waypoints.lane_waypoints[5].location.x
        target_y = self.lane_waypoints.lane_waypoints[5].location.y
        target_theta = self.lane_waypoints.lane_waypoints[5].rotation.y*np.pi/180

        curr_x = self.curr_state.location.x
        curr_y = self.curr_state.location.y
        curr_theta = self.curr_state.rotation.z*np.pi/180
        curr_speed = np.sqrt(self.curr_state.velocity.x**2 + self.curr_state.velocity.y**2)

        xError = ((target_x - curr_x) * np.cos(curr_theta)) + ((target_y - curr_y) * np.sin(curr_theta))
        yError = ((target_x - curr_x) * np.sin(curr_theta) * -1) + ((target_y - curr_y) * np.cos(curr_theta))
        thetaError = target_theta - curr_theta
        vError = 10-curr_speed

        print(target_x, target_y, target_theta, curr_x, curr_y, curr_theta, curr_speed, xError, vError)

        k_s = 1
        k_ds = 1
        k_n = 1
        k_theta = 0.1

        v = xError*k_s + vError*k_ds
        delta = k_n*yError+k_theta*thetaError

        if delta > np.pi/6:
            delta = np.pi/6
        elif delta < -np.pi/6:
            delta = -np.pi/6

        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        newAckermannCmd.steering_angle = delta

        self.controlPub.publish(newAckermannCmd)

if __name__ == "__main__":
    rospy.init_node("model_dynamics")
    controller = ego_waypoint_follower()

    # rate = rospy.Rate(100)  # 100 Hz
        # while not rospy.is_shutdown():
        # rate.sleep() 
    rospy.spin()