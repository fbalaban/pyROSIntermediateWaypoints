#!/usr/bin/env python

import rospy
import tracker

from mavros_msgs.msg import State
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointPushRequest
from mavros_msgs.srv import WaypointSetCurrent

from sensor_msgs.msg import NavSatFix

def position_callback(data):

    global state
    global wp_plan
    global waypoint_iter
    global distance
    global integrated_area
    global integration_started
    global vasi_mikri
    global pr_closest_point

    if state == "AUTO":
        actual_position = tracker.waypoint(data.latitude, data.longitude)
        previous_wp = wp_plan[waypoint_iter]
        next_wp = wp_plan[waypoint_iter + 1]

        closest_point = tracker.get_closest_point_on_segment(previous_wp.latitude, previous_wp.longitude, next_wp.latitude, next_wp.longitude, data.latitude, data.longitude)

        # if waypoint_iter > 0:
        #     if integration_started == 0:
        #         rospy.loginfo("========= INTEGRATION STARTED =========")
        #         vasi_mikri = get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
        #         pr_closest_point = waypoint(latitude=closest_point.latitude, longitude=closest_point.longitude)
        #         integration_started = 1
        #     else:
        #         vasi_megali  = get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
        #         integrated_area +=  ((vasi_mikri + vasi_megali) * get_distance(pr_closest_point.latitude, pr_closest_point.longitude,
        #                                                                      closest_point.latitude, closest_point.longitude)) / 2
        #         vasi_mikri = vasi_megali
        #         pr_closest_point = closest_point

        bearing = tracker.find_bearing(previous_wp, next_wp)

        # the next is the second way of calculating the next waypoint, by including the distance from the actual position of the UAV
        #remaining_distance = distance - get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
        #c_wp = find_command_waypoint(remaining_distance, bearing, closest_point)

        c_wp = tracker.find_command_waypoint(distance, bearing, closest_point)

        if tracker.get_distance(data.latitude,data.longitude,next_wp.latitude,next_wp.longitude) < 0.04: # meters
            waypoint_iter += 1
            rospy.loginfo("Waypoint id: " + str(waypoint_iter+2))
            if waypoint_iter == 45:
                # introduce angle if needed from previous file
                rospy.loginfo("For distance of: " + str(distance) + "km the integrated area is: " +
                              str(integrated_area) + " sq.kilometers")
                exit(0)

        # CONSTRUCTING THE WP

        the_wp = Waypoint()
        the_wp.frame = 3
        the_wp.command = 16
        the_wp.is_current = 1
        the_wp.autocontinue = 1
        the_wp.param1 = 0
        the_wp.param2 = 10
        the_wp.param3 = 0
        the_wp.param4 = 0
        the_wp.x_lat = c_wp.latitude
        the_wp.y_long = c_wp.longitude
        the_wp.z_alt = 100

        the_home = Waypoint()
        the_home.frame = 0
        the_home.command = 22
        the_home.is_current = 0
        the_home.autocontinue = 1
        the_home.param1 = 25
        the_home.param2 = 0
        the_home.param3 = 0
        the_home.param4 = 0
        the_home.x_lat = 41.1393967
        the_home.y_long = -8.7332993
        the_home.z_alt = 0

        the_req = WaypointPushRequest()
        the_req.waypoints.insert(0,the_home)
        the_req.waypoints.insert(1,the_wp)

        # SENDING THE LIST
        rospy.loginfo(" Intermediate waypoint, latitude:  " + str(c_wp.latitude) + ", longitude: " + str(c_wp.longitude))
        command_service(the_req)
        current_service(1)

def update_mission_callback(data):

    global has_wp_plan
    if has_wp_plan == False:
        global wp_plan
        mavros_wp_list = data.waypoints
        wp_plan = []
        count = 0
        for each_wp in mavros_wp_list:
            print "Latitude: " + str(each_wp.x_lat) + ", longitude: " + str(each_wp.y_long)
            wp_plan.append(tracker.waypoint(latitude=each_wp.x_lat, longitude=each_wp.y_long))
            count = count+1
        print count
        count = 0
        print "Easy list"
        for norm_wp in wp_plan:
            print "Latitude: " + str(norm_wp.latitude) + ", longitude: " + str(norm_wp.longitude)
            count = count + 1
        print count
        has_wp_plan = True

def state_callback(data):

    global initial_run
    global state

    if initial_run == True:
        if data.mode == "AUTO":
            initial_run = False
            state = "AUTO"
            print("Changed to AUTO")

waypoint_iter = 2
header_iter = 0

distance = 0.075 #  denoted in km
integrated_area = 0
integration_started = 0
vasi_mikri = 0
pr_closest_point = tracker.waypoint(0,0)

global initial_run
global state
global has_wp_plan

initial_run = True
state = "MANUAL"
has_wp_plan = False

if __name__ == '__main__':

    rospy.sleep(5)

    print "[INFO - Onboard Tracker] Just started"

    current_service = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    command_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    clearing_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

    rospy.init_node('onboard_tracker', anonymous=False)

    # Waiting for mode to change to AUTO
    rospy.Subscriber("mavros/state", State, state_callback)
    # Get the mission
    rospy.Subscriber("mavros/mission/waypoints", WaypointList, update_mission_callback)
    # Update vehicle position
    rospy.Subscriber("mavros/global_position/global", NavSatFix, position_callback)
    # clearing_service()
    rospy.spin()
