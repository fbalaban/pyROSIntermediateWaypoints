#!/usr/bin/env python
import rospy
import std_msgs.msg
from angle_calc import in_2d

# from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode

from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandInt
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import GlobalPositionTarget

from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointPushRequest
from mavros_msgs.srv import WaypointSetCurrent

from mavros_msgs.msg import Waypoint

from collections import namedtuple
from math import atan2,cos,sin,asin, radians, degrees, sqrt

# find bearing between two waypoints
# http://www.movable-type.co.uk/scripts/latlong.html (x and y are inverted)
def find_bearing(a, b):

    x_value = cos(radians(b.latitude)) * ( sin(radians(b.longitude) - radians(a.longitude)) )
    y_value = cos(radians(a.latitude)) * sin(radians(b.latitude)) - (sin(radians(a.latitude))  * cos(radians(b.latitude)) * cos(radians(b.longitude) - radians(a.longitude)))
    return degrees(atan2(x_value, y_value))

# find destination point given a distance and and bearing from a starting point
def find_command_waypoint(dis, bear, starting_waypoint):

    # d is the angular distance
    d = dis / R
    bearing = radians(bear)
    target_latitude = asin( (sin(radians(starting_waypoint.latitude)) * cos(d) ) + \
                            (cos(radians(starting_waypoint.latitude)) * sin(d) * cos(bearing)))
    target_longitude = radians(starting_waypoint.longitude) + \
                atan2(sin(bearing) * sin(d) * cos(radians(starting_waypoint.latitude)),
                      cos(d) - sin(radians(starting_waypoint.latitude)) * sin(target_latitude))
    return waypoint(latitude=round(degrees(target_latitude),7), longitude=round(degrees(target_longitude),7))

#http://www.java2s.com/Code/Java/2D-Graphics-GUI/Returnsclosestpointonsegmenttopoint.htm
def get_closest_point_on_segment(sx1, sy1, sx2, sy2, px, py):

    xDelta = sx2 - sx1
    yDelta = sy2 - sy1
    u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta)

    if u < 0:
      closestPoint = waypoint(sx1, sy1)
    elif u > 1:
      closestPoint = waypoint(sx2, sy2)
    else:
      closestPoint = waypoint(round((sx1 + u * xDelta),7), round((sy1 + u * yDelta),7))

    return closestPoint

def get_distance(lat1, lon1, lat2, lon2):

    f1 = radians(lat1)
    f2 = radians(lat2)
    df = radians(lat2-lat1)
    dl = radians(lon2-lon1)

    a = ( sin(df/2) * sin(df/2) ) + ( cos(f1) * cos(f2) * sin(dl/2) * sin(dl/2) )
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def set_mode_to_guided():
    #set mode to guided, once
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    set_mode_service(216,'GUIDED')

def set_mode_to_auto():
    #set mode to guided, once
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    set_mode_service(220,'AUTO')

# R is earth radius
R = 6371 # in km
# Initializing variables for the mission
waypoint = namedtuple('waypoint', 'latitude longitude')
wp_plan = [waypoint(37.5217000,-5.8576000),
			waypoint(37.5218571,-5.8578873),
			waypoint(37.5221571,-5.8573873),
			waypoint(37.5213571,-5.8577873),
			waypoint(37.5212238,-5.8562540),
			waypoint(37.5222333,-5.8553000),
			waypoint(37.5227667,-5.8548500),
			waypoint(37.5227667,-5.8539500),
			waypoint(37.5225500,-5.8524667),
			waypoint(37.5221370,-5.8508095),
			waypoint(37.5216537,-5.8502428),
			waypoint(37.5209870,-5.8501762),
			waypoint(37.5200948,-5.8506527),
			waypoint(37.5196782,-5.8514360),
			waypoint(37.5202967,-5.8524724),
			waypoint(37.5195000,-5.8529500),
			waypoint(37.5189750,-5.8530250),
			waypoint(37.5189250,-5.8540750),
			waypoint(37.5183750,-5.8541750),
			waypoint(37.5179667,-5.8551333),
			waypoint(37.5189714,-5.8559710),
			waypoint(37.5201000,-5.8555667),
			waypoint(37.5200667,-5.8574333),
			waypoint(37.5195000,-5.8576333),
			waypoint(37.5184547,-5.8576877),
			waypoint(37.5172167,-5.8572500),
			waypoint(37.5178214,-5.8568877),
			waypoint(37.5181714,-5.8559044),
			waypoint(37.5190380,-5.8570710),
			waypoint(37.5203000,-5.8566667),
			waypoint(37.5209571,-5.8569540),
			waypoint(37.5219238,-5.8565873),
			waypoint(37.5224667,-5.8561333),
			waypoint(37.5214667,-5.8538000),
			waypoint(37.5212139,-5.8523875),
			waypoint(37.5207504,-5.8519152),
			waypoint(37.5206569,-5.8508039),
			waypoint(37.5197865,-5.8521444),
			waypoint(37.5219370,-5.8518761),
			waypoint(37.5222667,-5.8535000),
			waypoint(37.5217000,-5.8576000)]

waypoint_iter = 0
header_iter = 0

distance = 0.04 #  denoted in km
integrated_area = 0
integration_started = 0
vasi_mikri = 0
pr_closest_point = waypoint(0,0)

# previous wp is the previous wp of the plan
previous_wp = wp_plan[waypoint_iter]
next_wp = wp_plan[waypoint_iter+1]
#tested_angle  = in_2d(wp_plan[2].latitude, wp_plan[2].longitude, wp_plan[3].latitude, wp_plan[3].longitude, wp_plan[4].latitude, wp_plan[4].longitude)

#rospy.loginfo(" Testing angle of : " + str(tested_angle) + " degrees")

#pub = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)

# if waypoint_iter is 3 (the one before the last), start measuring the area for integration

def callback(data):

    global wp_plan
    global waypoint_iter
    global distance
    global integrated_area
    global integration_started
    global vasi_mikri
    global pr_closest_point

    actual_position = waypoint(data.latitude, data.longitude)
    previous_wp = wp_plan[waypoint_iter]
    next_wp = wp_plan[waypoint_iter + 1]

    closest_point = get_closest_point_on_segment(previous_wp.latitude, previous_wp.longitude, next_wp.latitude, next_wp.longitude, data.latitude, data.longitude)

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

    bearing = find_bearing(previous_wp, next_wp)

    # the next is the second way of calculating the next waypoint, by including the distance from the actual position of the UAV
    #remaining_distance = distance - get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
    #c_wp = find_command_waypoint(remaining_distance, bearing, closest_point)

    c_wp = find_command_waypoint(distance, bearing, closest_point)

    if get_distance(data.latitude,data.longitude,next_wp.latitude,next_wp.longitude) < 0.04: # meters
        waypoint_iter += 1
        rospy.loginfo("Waypoint id: " + str(waypoint_iter+2))
        if waypoint_iter == 45:
            # introduce angle if needed from previous file
            rospy.loginfo("For distance of: " + str(distance) + "km the integrated area is: " +
                          str(integrated_area) + " sq.kilometers")
            exit(0)
    # SETTING THE SERVICE NEW WP

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


def listener():
    rospy.init_node('lat_lon_listener', anonymous=False)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.sleep(5)
    print "Just started"
    #set_mode_to_guided()
    set_mode_to_auto()
    # CLEARING THE PREVIOUS MISSION
    current_service = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    command_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    clearing_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    #rospy.loginfo(" Testing angle of : " + str(tested_angle) + " degrees")
    clearing_service()
    listener()
