#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL     = 4.0
STOP_BUFFER = 6.0


class WaypointUpdater(object):
  def __init__(self):

    # Properties
    self.prev_nrst_wp = 0 # total number of waypoints are 10902
    self.vehicle_pos = None
    self.current_linear_velocity = None
    self.upcoming_traffic_light_position = None
    self.waypoints = None
    self.braking = None
    self.decel = 1.0

    # subscribers
    rospy.init_node('waypoint_updater')
    rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
    rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
    rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
    rospy.Subscriber('/current_velocity', TwistStamped, self.crnt_vel_cb, queue_size=1)


    # publishers
    self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

    self.time_last_sample = rospy.rostime.get_time()


    rospy.spin()

  def pose_cb(self, msg):
    # # Log position change
    # if self.vehicle_pos != None:
    #     distance_change = math.sqrt((msg.pose.position.x - self.vehicle_pos.x)**2 + (msg.pose.position.y - self.vehicle_pos.y)**2)
    #     if distance_change > 2:
    #       print("Vehicle position: {}, {}".format(msg.pose.position.x, msg.pose.position.y))

    # current pose of the vehicle
    self.vehicle_pos = msg.pose.position

    # vehicle orientation in quanternions
    self.vehicle_orientation = msg.pose.orientation

  def crnt_vel_cb(self, msg):
    # vehicle velocity:
    self.current_vel = msg.twist.linear.x


  def waypoints_cb(self, waypoints):

    # TODO: Implement
    self.waypoints = waypoints
    if hasattr(self, 'vehicle_pos'):
    # try: # to catch when the error when self.vehicle_pos has not been created yet
      smallest_dist = float('inf')
      nearest_wp = 0

      # rospy.logwarn("previous nearest waypoint: %s", self.prev_nrst_wp)

      self.wp_num = len(waypoints.waypoints)
      dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
      hd = lambda a, b: math.atan2((b.y-a.y), (b.x-a.x))
      

      for i in xrange(self.prev_nrst_wp, self.wp_num):
        
        wp_pos = waypoints.waypoints[i].pose.pose.position

        # distance between vehichle and the nearest waypoint
        dist = dl(self.vehicle_pos, wp_pos)

        if dist < smallest_dist:
          nearest_wp = i
          smallest_dist = dist

      q = self.vehicle_orientation
      theta = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[-1]
      heading =  hd(self.vehicle_pos, wp_pos)
      angle = abs(theta - heading)

      if (angle>math.pi/4.0):
        nearest_wp += 1


      final_wps = self.get_final_wps(waypoints, nearest_wp)
      self.prev_nrst_wp = nearest_wp - 5

      # only set velocity in the fianl waypoints::::
      # the next part should only be done if there is a red light
      # set tl_waypoint velocity to 0
      # rospy.logwarn("traffic callback: %s", self.tl_waypoint)
      if self.tl_waypoint != -1:
        # self.tl_waypoint += 15 # back up a little bit ahead of traffic light
        rospy.logwarn("STOP!...")
        
        tl_waypoint_in_final_wp = self.tl_waypoint - nearest_wp

        for i in range(tl_waypoint_in_final_wp):
          # set the velocity at the traffic light to zero:
          # distance between vehicle waypoint and the traffic light waypoint
          if i < tl_waypoint_in_final_wp:
            distance = self.distance(final_wps.waypoints, i, tl_waypoint_in_final_wp)

            time_elapsed = rospy.rostime.get_time() - self.time_last_sample
            # acc = self.current_vel/(time_elapsed*distance)
            new_vel = math.sqrt(2 * 0.3 * (distance))

            # new_vel = (self.current_vel*self.current_vel * 0.01)/(2 * distance)


            self.time_last_sample = rospy.rostime.get_time()

            self.set_waypoint_velocity(final_wps.waypoints, 0, new_vel)

            # for debug
            # if i == 0:
            #   rospy.logwarn("NEW TARGET VEL: %s", new_vel)
          else:
            self.set_waypoint_velocity(final_wps.waypoints, i, 0)                    


      # rospy.logwarn("new target velocity: %s", new_vel)
        

      # the next condition might not work for arbitrary non-cyclic destinations
      if nearest_wp > (self.wp_num-100):
        self.prev_nrst_wp = 0
      

      current_vel = self.get_waypoint_velocity(final_wps.waypoints[0])
      # rospy.logwarn("current waypoint velocity: %s", current_vel)
      # rospy.logwarn("nearest waypoint: %s", nearest_wp)

      self.vehicle_wp = nearest_wp


      # publish final waypoints only if vehicle position has been received
      self.final_waypoints_pub.publish(final_wps)


  def get_final_wps(self, waypoints, nearest_wp):
    new_wp_lane = Lane()
    for i in xrange(nearest_wp,nearest_wp+LOOKAHEAD_WPS):
      if i >= self.wp_num:
        break
      new_wp_lane.waypoints.append(waypoints.waypoints[i])
    return new_wp_lane
    

  def traffic_cb(self, msg):
    # callback for /traffic_waypoint
    self.tl_waypoint = msg.data


  def obstacle_cb(self, msg):
    # TODO: Callback for /obstacle_waypoint message. We will implement it later
    pass


  def get_waypoint_velocity(self, waypoint):
    return waypoint.twist.twist.linear.x


  def set_waypoint_velocity(self, waypoints, waypoint, velocity):
    waypoints[waypoint].twist.twist.linear.x = velocity


  def distance(self, waypoints, wp1, wp2):
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    for i in range(wp1, wp2+1):
      dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
      wp1 = i
    return dist


if __name__ == '__main__':
  try:
    WaypointUpdater()
  except rospy.ROSInterruptException:
    rospy.logerr('Could not start waypoint updater node.')
