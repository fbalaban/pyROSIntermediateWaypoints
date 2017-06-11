#!/usr/bin/env python
import rospy
import std_msgs.msg

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
wp_plan = [waypoint(41.1427581, -8.7249470),
            waypoint(41.1393661, -8.717351),
            waypoint(41.1394005, -8.7103996),
            waypoint(41.1393661, -8.7032747),
            waypoint(41.1337729, -8.7032318)]

waypoint_iter = 1
header_iter = 0

distance = 0.180 #  denoted in km

# previous wp is the previous wp of the plan
previous_wp = wp_plan[waypoint_iter]
next_wp = wp_plan[waypoint_iter+1]

#pub = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)


def callback(data):

    global wp_plan
    global waypoint_iter
    global distance
    actual_position = waypoint(data.latitude, data.longitude)
    previous_wp = wp_plan[waypoint_iter]
    next_wp = wp_plan[waypoint_iter + 1]

    # rospy.loginfo(rospy.get_caller_id())
    bearing = find_bearing(actual_position, next_wp)
    # rospy.loginfo(" UAV Bearing: " + str(bearing) + " distance (km): " + str(distance))
    # rospy.loginfo(" UAV position lat: " + str(data.latitude) + ", lon: " + str(data.longitude))
    # rospy.loginfo(" Segment between waypoint A: lat: " + str(previous_wp.latitude) + ", lon: " + str(previous_wp.longitude))
    # rospy.loginfo("             and waypoint B: lat: " + str(next_wp.latitude) + ", lon: " + str(next_wp.longitude))
    closest_point = get_closest_point_on_segment(previous_wp.latitude, previous_wp.longitude, next_wp.latitude, next_wp.longitude, data.latitude, data.longitude)
    # rospy.loginfo(" closest point between that segment and UAV position is: lat: " + str(closest_point.latitude) + ", lon: " + str(closest_point.longitude))
    bearing = find_bearing(previous_wp, next_wp)
    # rospy.loginfo(" Bearing between waypoints: " + str(bearing))
    c_wp = find_command_waypoint(distance, bearing, closest_point)
    # rospy.loginfo(" .. and 100m far from it, towards (or exceeding) waypoint B,")
    rospy.loginfo("command point lat: " + str(c_wp.latitude) + " lon: " + str(c_wp.longitude))
    #
    # rospy.loginfo(" The distance between A and B: " + str(get_distance(previous_wp.latitude,previous_wp.longitude,next_wp.latitude,next_wp.longitude)) )
    # rospy.loginfo(" The distance between UAV position and A: " + str(get_distance(data.latitude,data.longitude,previous_wp.latitude,previous_wp.longitude)) )
    # rospy.loginfo(" The distance between UAV position and B: " + str(get_distance(data.latitude,data.longitude,next_wp.latitude,next_wp.longitude)) )
    # rospy.loginfo(" The distance between UAV position and command point: " + str(get_distance(data.latitude, data.longitude, c_wp.latitude, c_wp.longitude)))
    # rospy.loginfo(" The distance between closest position and command point: " + str(get_distance(closest_point.latitude, closest_point.longitude, c_wp.latitude, c_wp.longitude)))

    if get_distance(data.latitude,data.longitude,next_wp.latitude,next_wp.longitude) < 0.04: # meters
        waypoint_iter += 1
        rospy.loginfo("Waypoint id: " + str(waypoint_iter))

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
    the_home.x_lat = 41.1393929
    the_home.y_long = -8.7332993
    the_home.z_alt = 0

    the_req = WaypointPushRequest()
    the_req.waypoints.insert(0,the_home)
    the_req.waypoints.insert(1,the_wp)
    # SENDING THE LIST
    command_service(the_req)
    current_service(1)
    # rospy.sleep(1)
    # set_mode_to_auto()
    # #----------------- FIRST WAY ------------------------------------------------------------------------------------
    # # construct and call service for guided mode
    # command_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
    # command_service(0,16,1,0,0,0,0,c_wp.latitude,c_wp.longitude,105)
    # #---------------------------------------------------------------------------------------------------------------

    # #----------------- SECOND WAY ------------------------------------------------------------------------------------
    # command_service = rospy.ServiceProxy('mavros/cmd/command_int', CommandInt)
    # command_service(1,0,16,1,1,0,0,0,0,c_wp.latitude,c_wp.longitude,105)
    # #---------------------------------------------------------------------------------------------------------------

    # # -------------- THIRD WAY -------------------------------------------------------------------------------------
    # # HEADER CONSTRUCTION
    # the_header = std_msgs.msg.Header()
    # the_header.stamp = rospy.Time.now()
    #
    #
    # velocity = Vector3()
    # velocity.x = 0.8
    # velocity.y = 0.6
    # velocity.z = 0
    # acceleration_or_force = Vector3()
    # acceleration_or_force.x = 0.8
    # acceleration_or_force.y = 0.6
    # acceleration_or_force.z = 0
    # # TOPIC PUBLISHING
    # pub.publish(the_header,5,0b000000000000,c_wp.latitude,c_wp.longitude,105,velocity,acceleration_or_force,0.5,0.3)
    # #---------------------------------------------------------------------------------------------------------------
    # rospy.loginfo("lat: " + str(c_wp.latitude) + ", lon:" + str(c_wp.longitude) )

def listener():
    rospy.init_node('lat_lon_listener', anonymous=False)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "only once"
    #set_mode_to_guided()
    set_mode_to_auto()
    # CLEARING THE PREVIOUS MISSION
    current_service = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    command_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    clearing_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    clearing_service()
    listener()


